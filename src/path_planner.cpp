#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "icuas25_msgs/srv/path_service.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <mutex>

using std::placeholders::_1;
using std::placeholders::_2;

class PathPlanner : public rclcpp::Node
{
public:
  PathPlanner()
  : Node("path_planner"), raio_suporte_(5.0)
  {
    // Inicialização dos membros da classe
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/ghost/octomap", 10,
      std::bind(&PathPlanner::octomap_callback, this, _1));

    service_ = this->create_service<icuas25_msgs::srv::PathService>(
      "plan_path",
      std::bind(&PathPlanner::handle_service, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Path planning node initialized");
  }

private:
  // Membros da classe corretamente declarados
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Service<icuas25_msgs::srv::PathService>::SharedPtr service_;
  
  std::mutex octree_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;
  const double raio_suporte_;

  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(octree_mutex_);
    
    try {
        // Método correto para conversão de mensagem ROS para OcTree
         std::shared_ptr<octomap::OcTree> octree_(
            dynamic_cast<octomap::OcTree*>(
              octomap_msgs::fullMsgToMap(*msg)
            )
        );

        if(octree_){
            RCLCPP_DEBUG(this->get_logger(), "Octomap atualizado. Resolução: %.3f", 
                        octree_->getResolution());
        }
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Erro na conversão do Octomap: %s", e.what());
    }
  }

  void handle_service(
    const icuas25_msgs::srv::PathService::Request::SharedPtr request,
    icuas25_msgs::srv::PathService::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(octree_mutex_);
    
    if (!octree_) {
        RCLCPP_ERROR(this->get_logger(), "Octomap não disponível!");
        return;
    }

    // Converter pontos ROS para octomap::point3d
    octomap::point3d origin(request->origin.x, request->origin.y, request->origin.z);
    octomap::point3d destination(request->destination.x, request->destination.y, request->destination.z);
    octomap::point3d support(request->support.x, request->support.y, request->support.z);

    // Parâmetros da busca
    const double passo = 0.5;
    bool path_found = false;
    octomap::point3d melhor_ponto;

    octomap::point3d direcao = support - origin;
    double distancia_total = direcao.norm();
    direcao.normalize();

    double t_min = std::max(0.0, 1.0 - (raio_suporte_ / distancia_total));
    
    for (double t = 1.0; t >= t_min; t -= passo / distancia_total) {
        octomap::point3d ponto_atual = origin + direcao * (t * distancia_total);

        if ((ponto_atual - support).norm() > raio_suporte_) continue;

        octomap::point3d direcao_pd = destination - ponto_atual;
        double distancia_pd = direcao_pd.norm();
        direcao_pd.normalize();

        octomap::point3d fim_raio;
        bool colisao = octree_->castRay(ponto_atual, direcao_pd, fim_raio, true, distancia_pd);

        if (!colisao || (fim_raio - destination).norm() < 0.1) {
            melhor_ponto = ponto_atual;
            path_found = true;
            break;
        }
    }

    if (path_found) {
        response->path.x = melhor_ponto.x();
        response->path.y = melhor_ponto.y();
        response->path.z = melhor_ponto.z();
    } else {
        response->path.x = request->support.x;
        response->path.y = request->support.y;
        response->path.z = request->support.z;
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
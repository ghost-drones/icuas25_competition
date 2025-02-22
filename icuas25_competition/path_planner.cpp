#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "icuas25_competition/srv/path_service.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class PathPlanner : public rclcpp::Node
{
public:
  PathPlanner()
  : Node("path_planner"), current_map_(nullptr)
  {
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Service<path_planner::srv::PathService>::SharedPtr service_;
    // Subscriber to Octomap
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/ghost/octomap", 10,
      std::bind(&PathPlanner::octomap_callback, this, _1));

    // The Path Planning Service
    service_ = this->create_service<path_planner::srv::PathService>(
      "plan_path",
      std::bind(&PathPlanner::handle_service, this, _1, _2));

    std::shared_ptr<octomap::OcTree> octree_;
    double raio_support_;
    std::mutex octree_mutex_;

    RCLCPP_INFO(this->get_logger(), "Path planning node initialized");
    
  }

private:
  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(octree_mutex_);
    
    try {
        // Converter a mensagem ROS para OcTree
        std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap::AbstractOcTree::createMsg(msg->data));
        octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(abstract_tree.release()));
        
        if(octree_){
            RCLCPP_DEBUG(this->get_logger(), "Octomap atualizado. Resolução: %.3f", octree_->getResolution());
        }
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Erro na conversão do Octomap: %s", e.what());
    }
  } 

  void handle_service(
    const path_planner::srv::PathService::Request::SharedPtr request,
    path_planner::srv::PathService::Response::SharedPtr response)
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
    const double passo = 0.5;  // Passo de 0.5 metros para verificação
    bool path_found = false;
    octomap::point3d melhor_ponto;

    // 1. Gerar pontos ao longo da linha origin-support dentro da esfera
    octomap::point3d direcao = support - origin;
    double distancia_total = direcao.norm();
    direcao.normalize();

    // 2. Calcular intervalo válido de t dentro da esfera
    double t_min = std::max(0.0, 1.0 - (raio_support_ / distancia_total));
    
    // 3. Buscar do ponto mais próximo do support para trás
    for (double t = 1.0; t >= t_min; t -= passo / distancia_total) {
        octomap::point3d ponto_atual = origin + direcao * (t * distancia_total);

        // 4. Verificar se o ponto está dentro da esfera
        if ((ponto_atual - support).norm() > raio_support_) continue;

        // 5. Verificar linha do ponto atual até o destination
        octomap::point3d direcao_pd = destination - ponto_atual;
        double distancia_pd = direcao_pd.norm();
        direcao_pd.normalize();

        // 6. Verificar colisões ao longo do path
        octomap::point3d fim_raio;
        bool colisao = octree_->castRay(ponto_atual, direcao_pd, fim_raio, true,distancia_pd);

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
        response->path.x = (request->support.x);
        response->path.y = (request->support.y);
        response->path.z = (request->support.z);
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
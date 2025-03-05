#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "icuas25_msgs/srv/path_service.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <vector>
#include <mutex>
#include <sstream>
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;

class PathPlanner : public rclcpp::Node
{
public:
  PathPlanner()
  : Node("path_planner"), max_distance_(20.0)
  {
    // Inicialização dos membros da classe
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/ghost/octomap_inflated", 10,
        std::bind(&PathPlanner::octomap_callback, this, _1));

    service_ = this->create_service<icuas25_msgs::srv::PathService>(
        "/ghost/path_planner",
        std::bind(&PathPlanner::handle_service, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Path planning node initialized");
  }

private:
  // Membros da classe
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Service<icuas25_msgs::srv::PathService>::SharedPtr service_;
  
  std::mutex octree_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;
  const double max_distance_;

  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
      std::lock_guard<std::mutex> lock(octree_mutex_);
      
      try {
          octree_ = std::shared_ptr<octomap::OcTree>(
              dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg))
          );

          if (octree_) {
              RCLCPP_DEBUG(this->get_logger(), "Octomap atualizado. Resolução: %.3f", 
                           octree_->getResolution());
          }
          else {
              RCLCPP_ERROR(this->get_logger(), "Octomap NULO!!");
          }
      } 
      catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "Erro na conversão do Octomap: %s", e.what());
      }
  }

  // --- Funções auxiliares ---
  
  // 1. Verifica se há linha de visão livre entre dois pontos (usa castRay)
  bool isLineOfSightFree(const octomap::point3d &origin, const octomap::point3d &target)
  {
      octomap::point3d diff = target - origin;
      double distance = diff.norm();
      if (distance < 1e-6)
          return true; // Pontos praticamente coincidentes

      // Utiliza a multiplicação de vetor por escalar (vetor * double)
      octomap::point3d direction = diff * (1.0 / distance);
      octomap::point3d hit;
      // castRay retorna true se encontrar um obstáculo; portanto, invertemos o resultado
      return !octree_->castRay(origin, direction, hit, false, distance);
  }
  
  // 2. Verifica se dois pontos (p1 e p2) são visíveis a partir de um observador (support)
  bool arePointsVisibleFrom(const octomap::point3d &observer, const octomap::point3d &p1, const octomap::point3d &p2)
  {
      return isLineOfSightFree(observer, p1) && isLineOfSightFree(observer, p2);
  }
  
  // 3. Verifica se o segmento entre p1 e p2 está livre de colisões (verifica cada ponto amostrado)
  bool isSegmentCollisionFree(const octomap::point3d &p1, const octomap::point3d &p2, double step_size)
  {
      double dist = (p2 - p1).norm();
      int num_steps = std::ceil(dist / step_size);
      for (int i = 0; i <= num_steps; ++i)
      {
          double t = static_cast<double>(i) / num_steps;
          octomap::point3d pt = p1 * (1.0 - t) + p2 * t;
          octomap::OcTreeNode* node = octree_->search(pt);
          if (node && octree_->isNodeOccupied(node))
          {
              return false;  // Colisão detectada
          }
      }
      return true;
  }
  
  // 4. Verifica se todos os pontos do segmento entre p1 e p2 são visíveis a partir do support
  bool isSegmentVisibleFromSupport(const octomap::point3d &p1, const octomap::point3d &p2, 
                                   const octomap::point3d &support, double step_size)
  {
      double dist = (p2 - p1).norm();
      int num_steps = std::ceil(dist / step_size);
      for (int i = 0; i <= num_steps; ++i)
      {
          double t = static_cast<double>(i) / num_steps;
          octomap::point3d pt = p1 * (1.0 - t) + p2 * t;
          if (!isLineOfSightFree(support, pt))
          {
              return false;  // Algum ponto não está visível para o support
          }
      }
      return true;
  }
  
  std::vector<octomap::point3d> generate_points_along_line(
      const octomap::point3d& start, 
      const octomap::point3d& end, 
      double step_size)
  {
      std::vector<octomap::point3d> points;
      octomap::point3d direction = end - start;
      double distance = direction.norm();
      
      if (distance < 1e-6) {
          points.push_back(start);
          return points;
      }
      
      direction = direction * (1.0 / distance); // Normaliza
      
      for (double t = 0.0; t <= distance; t += step_size) {
          points.push_back(start + direction * t);
      }
      
      return points;
  }
  // --- Fim das funções auxiliares ---

  void handle_service(
      const icuas25_msgs::srv::PathService::Request::SharedPtr request,
      icuas25_msgs::srv::PathService::Response::SharedPtr response)
  {
      // Garantir exclusão mútua durante o acesso à octree
      std::lock_guard<std::mutex> lock(octree_mutex_);
      
      // Validação inicial: verifica se a octree está disponível
      if (!octree_) {
          RCLCPP_ERROR(this->get_logger(), "Octomap não disponível!");
          response->path = request->support;
          return;
      }
      // Converter pontos ROS para octomap::point3d
      octomap::point3d origin_oct(request->origin.x, request->origin.y, request->origin.z);
      octomap::point3d destination_oct(request->destination.x, request->destination.y, request->destination.z);
      octomap::point3d support_oct(request->support.x, request->support.y, request->support.z);

      std::ostringstream saida;
      saida << "x:" << request->origin.x << " y:" << request->origin.y << " z:" << request->origin.z;
      RCLCPP_ERROR(this->get_logger(), "ORIGIN: %s", saida.str().c_str());

      // Parâmetros da busca
      bool path_found = false;
      double step_size = octree_->getResolution();

      // Variável que armazenará o melhor ponto encontrado
      octomap::point3d best_point;

      // Casos especiais: se origin ou destination coincidirem com support, retorna support
      if (origin_oct.distance(support_oct) < step_size || destination_oct.distance(support_oct) < step_size) {
          best_point = support_oct;
          path_found = true;
      } else {
          // Gera os pontos ao longo da linha entre origin e support
          std::vector<octomap::point3d> points_list = generate_points_along_line(origin_oct, support_oct, step_size / 2);
          
          for (const auto &candidate : points_list) {
              RCLCPP_DEBUG(this->get_logger(), "candidate: %.2f %.2f %.2f", candidate.x(), candidate.y(), candidate.z());

              // Se o candidato for o próprio support, avalia a linha de visão entre support e destination
              if (candidate == support_oct) {
                  if (isLineOfSightFree(support_oct, destination_oct)) {
                      best_point = support_oct;
                      path_found = true;
                  }
                  break;
              }

              // Verifica se o caminho entre o candidato e o destination está livre de colisões
              if (!isSegmentCollisionFree(candidate, destination_oct, step_size)) {
                  RCLCPP_DEBUG(this->get_logger(), "Colisão detectada de candidato para destino");
                  continue;
              }
              
              // Verifica se todo o segmento candidato -> destination é visível a partir do support
              if (!isSegmentVisibleFromSupport(candidate, destination_oct, support_oct, step_size)) {
                  RCLCPP_DEBUG(this->get_logger(), "Segmento não visível do support");
                  continue;
              }
              
              // Se passar em todas as verificações, o candidato é considerado válido
              best_point = candidate;
              path_found = true;
              break; // Pára no primeiro candidato válido (mais próximo da origem)
          }
          
          if (path_found) {
              saida.str(""); 
              saida << "x:" << best_point.x() << " y:" << best_point.y() << " z:" << best_point.z();
              RCLCPP_ERROR(this->get_logger(), "BEST POINT: %s", saida.str().c_str());
              response->path.x = best_point.x();
              response->path.y = best_point.y();
              response->path.z = best_point.z();
          } else {
              response->path = request->support;
          }
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
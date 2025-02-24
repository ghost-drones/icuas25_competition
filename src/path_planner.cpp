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
  : Node("path_planner"), support_radius_(5)
  {
    // Inicialização dos membros da classe
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/ghost/octomap", 10,
        std::bind(&PathPlanner::octomap_callback, this, _1));

    service_ = this->create_service<icuas25_msgs::srv::PathService>(
        "/ghost/path_planner",
        std::bind(&PathPlanner::handle_service, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Path planning node initialized");
  }

private:
  // Membros da classe corretamente declarados
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Service<icuas25_msgs::srv::PathService>::SharedPtr service_;
  
    std::mutex octree_mutex_;
    std::shared_ptr<octomap::OcTree> octree_;
    const unsigned support_radius_;

    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        
        try {
            octree_ = std::shared_ptr<octomap::OcTree>(
            dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg))
            );

            if(octree_){
                RCLCPP_DEBUG(this->get_logger(), "Octomap atualizado. Resolução: %.3f", 
                            octree_->getResolution());
            }
            else{
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
        if(distance < 1e-6) 
            return true; // Pontos praticamente coincidentes

        octomap::point3d direction = diff * static_cast<float>(1.0 / distance);
        octomap::point3d hit;
        // castRay retorna true se encontrar obstáculo; por isso invertemos o resultado
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
    // --- Fim das funções auxiliares ---

    void handle_service(
        const icuas25_msgs::srv::PathService::Request::SharedPtr request,
        icuas25_msgs::srv::PathService::Response::SharedPtr response)
    {
        // start validations
        std::lock_guard<std::mutex> lock(octree_mutex_);
        // Parametros de inicio
        // Valida se a octree está disponível
        if (!octree_) {
            RCLCPP_ERROR(this->get_logger(), "Octomap não disponível!");
            response->path = request->support;
            return;
        }
        // Converter pontos ROS para octomap::point3d
        octomap::point3d origin_oct(request->origin.x, request->origin.y, request->origin.z);
        octomap::point3d destination_oct(request->destination.x, request->destination.y, request->destination.z);
        octomap::point3d support_oct(request->support.x, request->support.y, request->support.z);
        
        // Parâmetros da busca
        const unsigned step_voxel = 1;
        bool path_found = false;

        double resolution = octree_->getResolution();
        double max_distance = support_radius_ * resolution;
        double step_size = step_voxel * resolution;

        // Valida se origin e destination estão dentro do raio de atuação do support
        if (origin_oct.distance(support_oct) > max_distance) {
            RCLCPP_ERROR(this->get_logger(), "Origin fora da área do suporte!: %f", max_distance);
            response->path = request->support;
            return;
        }
        if (destination_oct.distance(support_oct) > max_distance) {
            RCLCPP_ERROR(this->get_logger(), "Destination fora da área do suporte!");
            response->path = request->support;
            return;
        }
        
        // Valida se, mesmo estando dentro do raio, origin e destination são visíveis a partir do support
        if (!arePointsVisibleFrom(support_oct, origin_oct, destination_oct)) {
            RCLCPP_ERROR(this->get_logger(), "Origin ou Destination não são visíveis a partir do support!");
            response->path = request->support;
            return;
        }
        // end validations

        // Parâmetros da retorno
        octomap::point3d best_point;

        // start kernel code 

        // Casos especiais: se origin ou destination coincidirem com support, retorna support
        if ((origin_oct - support_oct).norm() < resolution || (destination_oct - support_oct).norm() < resolution) {
            best_point = support_oct;
            path_found = true;
        } else {
            // Busca pelo best_point ao longo da reta entre origin e support
            octomap::KeyRay ray;
            octree_->computeRayKeys(origin_oct, support_oct, ray);
            octomap::OcTreeKey support_key = octree_->coordToKey(support_oct);

            for (const auto & key : ray) {
                octomap::point3d candidate = octree_->keyToCoord(key);
                
                // Se o candidato for o próprio support, avalia a linha de visão entre support e destination
                if (key == support_key) {
                    if (isLineOfSightFree(support_oct, destination_oct)) {
                        best_point = support_oct;
                        path_found = true;
                    }
                    break;
                }

                // Verifica se o caminho entre o candidato e o destination está livre de colisões
                if (!isSegmentCollisionFree(candidate, destination_oct, step_size))
                continue;
            
                // Verifica se todo o segmento candidato -> destination é visível a partir do support
                if (!isSegmentVisibleFromSupport(candidate, destination_oct, support_oct, step_size))
                    continue;
                
                // Se passar nas duas verificações, candidate é considerado válido
                best_point = candidate;
                path_found = true;
                break;
            }
        }
        if (path_found) {
            response->path.x = best_point.x();
            response->path.y = best_point.y();
            response->path.z = best_point.z();
        } else {
            response->path = request->support;
            RCLCPP_ERROR(this->get_logger(), "Não encontrado path adequado");
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
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

    void handle_service(
        const icuas25_msgs::srv::PathService::Request::SharedPtr request,
        icuas25_msgs::srv::PathService::Response::SharedPtr response)
    {
        // start validations
        std::lock_guard<std::mutex> lock(octree_mutex_);
        // Parametros de inicio
        if (!octree_) {
            RCLCPP_ERROR(this->get_logger(), "Octomap não disponível!");
            response->path.x = request->support.x;
            response->path.y = request->support.y;
            response->path.z = request->support.z;
            return;
        }

        octomap::point3d origin_oct(request->origin.x, request->origin.y, request->origin.z);
        octomap::point3d destination_oct(request->destination.x, request->destination.y, request->destination.z);
        octomap::point3d support_oct(request->support.x, request->support.y, request->support.z);

        // Verifica se origin está dentro do raio
        if (origin_oct.distance(support_oct) > max_distance) {
            RCLCPP_ERROR(this->get_logger(), "Origin fora da área do suporte!");
            response->path.x = request->support.x;
            response->path.y = request->support.y;
            response->path.z = request->support.z;
            return;
        }

        // Verifica se destination está dentro do raio
        if (destination_oct.distance(support_oct) > max_distance) {
            RCLCPP_ERROR(this->get_logger(), "Destination fora da área do suporte!");
            response->path.x = request->support.x;
            response->path.y = request->support.y;
            response->path.z = request->support.z;
            return;
        }
        // end validations
        

        // Parâmetros da busca
        // step e path_found
        double resolution = octree_->getResolution();
        double max_distance = support_radius_ * resolution;
        const unsigned step = 1;
        bool path_found = false;
        
        
        // Parâmetros da retorno
        octomap::point3d best_point;

        // start kernel code 

        // Verifica se origin ou destination coincidem com o support (casos especiais)
        if (origin.distance(support) < octree_->getResolution()) {
            // Origin é o próprio support
            best_point = support;
            path_found = true;
        } else if (destination.distance(support) < octree_->getResolution()) {
            // Destination é o support, verifica caminho do origin para support
            best_point = support;
            path_found = true;
        } else {
            // Procede com a busca normal
            octomap::KeyRay ray;
            octree_->computeRayKeys(origin, support, ray);
            
            octomap::OcTreeKey support_key = octree_->coordToKey(support);
            
            for (const auto& key : ray) {
                octomap::point3d candidate = octree_->keyToCoord(key);
                
                // Pula o candidate se for além do suporte (edge case)
                if (key == support_key) {
                    // Verifica linha do support para destination
                    octomap::point3d dir_dest = destination - support;
                    float max_dist = dir_dest.norm();
                    dir_dest.normalize();
                    octomap::point3d hit;
                    bool hit_obs = octree_->castRay(support, dir_dest, hit, false, max_dist);
                    
                    if (!hit_obs) {
                        best_point = support;
                        path_found = true;
                    }
                    break;
                }
                
                // Verifica caminho do candidate para destination
                octomap::point3d dir_dest = destination - candidate;
                float max_dist_dest = dir_dest.norm();
                dir_dest.normalize();
                octomap::point3d hit_dest;
                bool blocked_dest = octree_->castRay(candidate, dir_dest, hit_dest, false, max_dist_dest);
                
                if (blocked_dest) continue;
                
                // Verifica visibilidade do support para o candidate
                octomap::point3d dir_support = candidate - support;
                float max_dist_support = dir_support.norm();
                dir_support.normalize();
                octomap::point3d hit_support;
                bool blocked_support = octree_->castRay(support, dir_support, hit_support, false, max_dist_support);
                
                if (!blocked_support) {
                    best_point = candidate;
                    path_found = true;
                    break;
                }
            }
        }
        // end kernel code

        if (path_found) {
            response->path.x = best_point.x();
            response->path.y = best_point.y();
            response->path.z = best_point.z();
        } else {
            response->path.x = request->support.x;
            response->path.y = request->support.y;
            response->path.z = request->support.z;
            RCLCPP_ERROR(this->get_logger(), "Não encontrado path");
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
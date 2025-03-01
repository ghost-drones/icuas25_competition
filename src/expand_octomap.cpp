#include <memory>
#include <chrono>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "octomap_msgs/msg/octomap.hpp"

#include "octomap_msgs/conversions.h"
#include <octomap/octomap.h>

using namespace std::chrono_literals;

class OctomapPublisher : public rclcpp::Node
{
public:
  OctomapPublisher()
  : Node("octomap_publisher")
  {
    this->declare_parameter<double>("obstacles_inflation", 0.3);

    client_ = this->create_client<octomap_msgs::srv::GetOctomap>("/octomap_full");

    publisher_normal_ = this->create_publisher<octomap_msgs::msg::Octomap>("/ghost/octomap", 1);
    publisher_inflated_ = this->create_publisher<octomap_msgs::msg::Octomap>("/ghost/octomap_inflated", 1);

    timer_ = this->create_wall_timer(
      1s, std::bind(&OctomapPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Serviço /octomap_full indisponível, aguardando...");
      return;
    }
    auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
    auto future_result = client_->async_send_request(
      request, std::bind(&OctomapPublisher::handle_service_response, this, std::placeholders::_1));
  }

  void handle_service_response(rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedFuture future)
  {
    try {
      auto response = future.get();
      process_octomap(response->map);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Falha na chamada do serviço: %s", e.what());
    }
  }

  void process_octomap(const octomap_msgs::msg::Octomap & octomap_msg)
  {
    // Publica o octomap original
    publisher_normal_->publish(octomap_msg);
    RCLCPP_INFO(this->get_logger(), "Octomap original publicado.");

    // Converte a mensagem para um octree
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(
      octomap_msgs::fullMsgToMap(octomap_msg));
    if (!tree) {
      RCLCPP_ERROR(this->get_logger(), "Falha ao converter a mensagem para octree.");
      return;
    }

    double obstacles_inflation = this->get_parameter("obstacles_inflation").as_double();
    double resolution = tree->getResolution();

    int inf_step = static_cast<int>(std::ceil(obstacles_inflation / resolution));

    // Cria um novo octree para o mapa inflado com a mesma resolução
    octomap::OcTree inflated_tree(resolution);

    // Para cada voxel ocupado no octree original, “infla” os obstáculos
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs();
         it != end; ++it) {
      if (it->getOccupancy() >= tree->getOccupancyThres()) {
        octomap::point3d center = it.getCoordinate();
        // Percorre uma vizinhança cúbica de lado 2*inf_step+1
        for (int dx = -inf_step; dx <= inf_step; ++dx) {
          for (int dy = -inf_step; dy <= inf_step; ++dy) {
            for (int dz = -inf_step; dz <= inf_step; ++dz) {
              octomap::point3d new_point(
                center.x() + dx * resolution,
                center.y() + dy * resolution,
                center.z() + dz * resolution);
              inflated_tree.updateNode(new_point, true);
            }
          }
        }
      }
    }

    octomap_msgs::msg::Octomap inflated_msg;
    inflated_msg.header = octomap_msg.header;
    inflated_msg.id = octomap_msg.id;
    inflated_msg.resolution = resolution;
    octomap_msgs::fullMapToMsg(inflated_tree, inflated_msg);

    publisher_inflated_->publish(inflated_msg);
    RCLCPP_INFO(this->get_logger(), "Octomap inflado publicado.");

    delete tree;
  }

  rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr publisher_normal_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr publisher_inflated_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

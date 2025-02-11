#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/ColorOcTree.h"
#include "octomap/OcTree.h"  // Necessário para usar OcTree
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <vector>
#include <cmath>

class OctomapColorizer : public rclcpp::Node {
public:
  OctomapColorizer() : Node("octomap_colorizer") {
    subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap", 10,
        std::bind(&OctomapColorizer::octomapCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<octomap_msgs::msg::Octomap>("/colored_octomap", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/network_markers", 10);
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    // Desserializa a mensagem Octomap
    std::shared_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
    if (!abstract_tree) {
      RCLCPP_ERROR(this->get_logger(), "Falha ao desserializar a mensagem Octomap");
      return;
    }

    // Tenta converter diretamente para ColorOcTree
    std::shared_ptr<octomap::ColorOcTree> color_tree = 
      std::dynamic_pointer_cast<octomap::ColorOcTree>(abstract_tree);
    if (!color_tree) {
      // Se não for ColorOcTree, converte para OcTree e copia os nós para uma nova ColorOcTree
      octomap::OcTree* tree_ptr = dynamic_cast<octomap::OcTree*>(abstract_tree.get());
      if (!tree_ptr) {
        RCLCPP_ERROR(this->get_logger(), "Octomap não é do tipo OcTree");
        return;
      }
      color_tree = std::make_shared<octomap::ColorOcTree>(tree_ptr->getResolution());
      // Copia os nós ocupados do OcTree original para a ColorOcTree (com cor branca)
      for (auto it = tree_ptr->begin_leafs(), end = tree_ptr->end_leafs(); it != end; ++it) {
        if (tree_ptr->isNodeOccupied(*it)) {
          octomap::OcTreeKey key = it.getKey();
          color_tree->updateNode(key, true);
          color_tree->setNodeColor(key, 255, 255, 255); // Branco
        }
      }
    }


    // Vetor para armazenar posições candidatas únicas
    std::vector<geometry_msgs::msg::Point> candidate_positions;
    // Distância mínima para considerar candidatos como distintos (em metros)
    double min_distance = 3.0;

    // Itera por cada voxel ocupado
    for (auto it = color_tree->begin_leafs(), end = color_tree->end_leafs(); it != end; ++it) {
      if (color_tree->isNodeOccupied(*it)) {
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();

        // Obtém a chave do voxel atual
        octomap::OcTreeKey key = it.getKey();

        // Vetor com os 6 deslocamentos correspondentes aos vizinhos em cada direção cartesiana
        // (note que estes deslocamentos são em "número de células")
        std::vector<octomap::OcTreeKey> directions = {
          octomap::OcTreeKey(1,0,0),
          octomap::OcTreeKey(-1,0,0),
          octomap::OcTreeKey(0,1,0),
          octomap::OcTreeKey(0,-1,0),
          octomap::OcTreeKey(0,0,1),
          octomap::OcTreeKey(0,0,-1)
        };

        // Para cada direção, se o vizinho estiver livre, gera uma posição candidata
        bool markerAddedForVoxel = false;
        for (auto &dir : directions) {
          // Calcula a chave do vizinho
          octomap::OcTreeKey neighbor_key(key[0] + dir[0],
                                          key[1] + dir[1],
                                          key[2] + dir[2]);
          // Se o vizinho não existir ou não estiver ocupado, considere-o livre
          octomap::ColorOcTreeNode* neighbor = color_tree->search(neighbor_key);
          bool is_free = (neighbor == nullptr) || !color_tree->isNodeOccupied(*neighbor);
          if (is_free) {
            // Calcula a posição candidata deslocada 3m na direção considerada
            geometry_msgs::msg::Point candidate;
            candidate.x = x + (dir[0] * 3.0);
            candidate.y = y + (dir[1] * 3.0);
            candidate.z = z + (dir[2] * 3.0);

            // Verifica se já existe um candidato próximo para evitar duplicatas
            bool too_close = false;
            for (const auto &pt : candidate_positions) {
              double dx = candidate.x - pt.x;
              double dy = candidate.y - pt.y;
              double dz = candidate.z - pt.z;
              double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
              if (dist < min_distance) {
                too_close = true;
                break;
              }
            }
            if (!too_close) {
              candidate_positions.push_back(candidate);
            }
            markerAddedForVoxel = true;
            break; // Adiciona apenas um marcador por voxel
          }
        }
      }
    }

    // Cria um MarkerArray para os candidatos (posições para o drone)
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 0;
    for (const auto &pt : candidate_positions) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "drone_positions";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = pt;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.5;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      // Cor verde para indicar as posições do drone
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker.lifetime = rclcpp::Duration::from_seconds(0);  // Fica ativo indefinidamente
      marker_array.markers.push_back(marker);
    }
    // Publica os marcadores
    marker_pub_->publish(marker_array);

    // --------------------------------------------------------------
    // Publica o Octomap colorido (como anteriormente)
    // --------------------------------------------------------------
    octomap_msgs::msg::Octomap colored_msg;
    colored_msg.header = msg->header;
    if (octomap_msgs::fullMapToMsg(*color_tree, colored_msg)) {
      publisher_->publish(colored_msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Falha ao serializar ColorOcTree para mensagem Octomap");
    }
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapColorizer>());
  rclcpp::shutdown();
  return 0;
}

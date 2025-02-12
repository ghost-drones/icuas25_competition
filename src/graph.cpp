#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <cmath>

// Estrutura para representar uma aresta do grafo
struct Edge {
  int source;
  int target;
  double weight;
};

class GraphBuilder : public rclcpp::Node {
public:
  GraphBuilder()
  : Node("graph_builder"), graph_counter_(0)
  {
    // Subscreve o tópico /drone_poses
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/drone_poses", 10,
      std::bind(&GraphBuilder::poseCallback, this, std::placeholders::_1));
    
    // Subscreve o tópico /octomap
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap", 10,
      std::bind(&GraphBuilder::octomapCallback, this, std::placeholders::_1));

    // Cria o publicador para sinalizar que o grafo foi finalizado, publicando um contador
    graph_pub_ = this->create_publisher<std_msgs::msg::Int32>("/graph_finished", 10);
  }

private:
  // Callback para receber a PoseArray
  void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    last_pose_array_ = msg;
    updateGraph();
  }

  // Callback para receber o Octomap
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    last_octomap_ = msg;
    updateGraph();
  }

  // Função para atualizar e construir o grafo quando ambas as mensagens estiverem disponíveis
  void updateGraph()
  {
    // Se não temos ambas as mensagens, não procede com a atualização
    if (!last_pose_array_ || !last_octomap_) {
      return;
    }

    // Converte a mensagem do Octomap para uma OcTree
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*last_octomap_);
    // Efetua o cast para octomap::OcTree (é importante verificar se a conversão foi bem-sucedida)
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    if (!octree) {
      RCLCPP_ERROR(this->get_logger(), "Falha na conversão da mensagem Octomap para OcTree.");
      return;
    }

    // Monta a lista de vértices: adiciona o ponto (0,0,0) e os pontos dos drones
    std::vector<octomap::point3d> vertices;
    vertices.push_back(octomap::point3d(0.0f, 0.0f, 0.0f));  // Adiciona a origem

    for (const auto & pose : last_pose_array_->poses) {
      vertices.push_back(octomap::point3d(pose.position.x, pose.position.y, pose.position.z));
    }

    // Vetor para armazenar as arestas do grafo
    std::vector<Edge> graph_edges;

    // Itera sobre todos os pares de vértices para verificar a linha de visão
    for (size_t i = 0; i < vertices.size(); ++i) {
      for (size_t j = i + 1; j < vertices.size(); ++j) {
        // Calcula a distância euclidiana entre os pontos
        double dist = (vertices[j] - vertices[i]).norm();
        if (dist < 1e-6) {  // Evita divisão por zero se os pontos coincidirem
          continue;
        }
        // Calcula o vetor direção normalizado de i para j
        octomap::point3d direction = vertices[j] - vertices[i];
        direction /= dist;
        octomap::point3d hit;

        // Utiliza castRay para verificar se há obstáculo entre vertices[i] e vertices[j]
        bool obstacle_hit = octree->castRay(vertices[i], direction, hit, dist);
        if (!obstacle_hit) {
          // Se não houve interseção com célula ocupada, adiciona a aresta
          graph_edges.push_back({static_cast<int>(i), static_cast<int>(j), dist});
        }
      }
    }

    // Incrementa o contador e publica a mensagem indicando quantas vezes o grafo foi criado
    graph_counter_++;
    std_msgs::msg::Int32 msg;
    msg.data = graph_counter_;
    graph_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Grafo finalizado. Contador: %d", graph_counter_);

    // Libera a memória alocada para a OcTree
    delete octree;
  }

  // Assinantes
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

  // Publicador para sinalizar que o grafo foi finalizado
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr graph_pub_;

  // Armazena as últimas mensagens recebidas
  geometry_msgs::msg::PoseArray::SharedPtr last_pose_array_;
  octomap_msgs::msg::Octomap::SharedPtr last_octomap_;

  // Contador de vezes que o grafo foi criado
  int graph_counter_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraphBuilder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>  // se desejar manter marcadores visuais
#include <tf2/LinearMath/Quaternion.h>

#include <queue>
#include <vector>
#include <unordered_map>
#include <tuple>
#include <limits>
#include <cmath>
#include <memory>
#include <algorithm>

// Estrutura para células da grade (usada no Dijkstra)
struct GridCell {
  int i, j;
  double dist;
};

// Comparador para a fila de prioridade (min-heap)
struct CompareGridCell {
  bool operator()(const GridCell &a, const GridCell &b) {
    return a.dist > b.dist;
  }
};

class TerritorialWatersNode : public rclcpp::Node
{
public:
  TerritorialWatersNode() : Node("territorial_waters_node")
  {
    // Parâmetros configuráveis:
    //   slice_thickness: espessura da fatia em z (em metros)
    //   layer_margin: quanto ampliar os limites (bounding box) de cada camada
    //   territorial_distance: distância usada para definir “territorial waters”
    slice_thickness_      = 2.8;  // por exemplo, 2.8 m de altura da fatia
    layer_margin_         = 5.0;  // amplia os limites em 5 metros
    territorial_distance_ = 3.0;  // 3 metros da costa
    percent_remaining_goals_ = 5;  // %

    // Subscreve o tópico do Octomap
    subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap", 1,
      std::bind(&TerritorialWatersNode::octomapCallback, this, std::placeholders::_1)
    );

    // Publica os resultados (lista de poses com posição e orientação)
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("drone_poses", 1);

    RCLCPP_INFO(this->get_logger(), "Nó territorial_waters_node iniciado!");
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    // Converte a mensagem para um octomap::OcTree
    std::shared_ptr<octomap::OcTree> tree(
      dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg))
    );
    if (!tree) {
      RCLCPP_ERROR(this->get_logger(), "Falha ao converter Octomap para OcTree");
      return;
    }
    double resolution = tree->getResolution();

    // Agrupa os nós folhas por camada (fatia em z, de acordo com slice_thickness_)
    // Para cada camada, armazenamos:
    //   - Um vetor de tuplas (x, y, ocupado) para os centros dos nós.
    //   - Os limites (bounding box) em x e y.
    std::unordered_map<int, std::vector<std::tuple<double, double, bool>>> layers;
    std::unordered_map<int, std::tuple<double, double, double, double>> layer_bounds; // min_x, max_x, min_y, max_y

    // Determina os limites do Octomap
    double min_x, min_y, min_z;
    tree->getMetricMin(min_x, min_y, min_z);

    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();

      // Calcula o índice da camada (fatia) usando slice_thickness_
      int layer = static_cast<int>(std::floor(z / slice_thickness_));
      bool occupied = tree->isNodeOccupied(*it);
      layers[layer].push_back(std::make_tuple(x, y, occupied));
      
      // Atualiza os limites da camada
      if (layer_bounds.find(layer) == layer_bounds.end()) {
        layer_bounds[layer] = std::make_tuple(x, x, y, y);
      } else {
        auto &bounds = layer_bounds[layer];
        std::get<0>(bounds) = std::min(std::get<0>(bounds), x); // min_x
        std::get<1>(bounds) = std::max(std::get<1>(bounds), x); // max_x
        std::get<2>(bounds) = std::min(std::get<2>(bounds), y); // min_y
        std::get<3>(bounds) = std::max(std::get<3>(bounds), y); // max_y
      }
    }

    // Inicializa a mensagem que conterá todas as poses (posição + orientação)
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "world";
    pose_array.header.stamp = this->get_clock()->now();

    // Identifica a camada mais baixa (geralmente correspondente ao chão) e ignora-a
    int min_layer = std::numeric_limits<int>::max();
    for (const auto &layer_pair : layers) {
      min_layer = std::min(min_layer, layer_pair.first);
    }

    // Processa cada camada (exceto a mais baixa)
    for (auto &layer_pair : layers)
    {
      int layer = layer_pair.first;
      if (layer == min_layer)
        continue;  // ignora a camada inferior

      auto &points = layer_pair.second;
      if (layer_bounds.find(layer) == layer_bounds.end())
        continue;
      
      double min_x_layer, max_x_layer, min_y_layer, max_y_layer;
      std::tie(min_x_layer, max_x_layer, min_y_layer, max_y_layer) = layer_bounds[layer];
      
      // Aumenta os limites da camada de acordo com layer_margin_
      min_x_layer -= layer_margin_;
      max_x_layer += layer_margin_;
      min_y_layer -= layer_margin_;
      max_y_layer += layer_margin_;
      
      // Define a grade 2D com passo igual à resolução do octomap
      int grid_width  = static_cast<int>(std::ceil((max_x_layer - min_x_layer) / resolution));
      int grid_height = static_cast<int>(std::ceil((max_y_layer - min_y_layer) / resolution));
      
      // Cria a grade de ocupação (inicialmente "livre")
      std::vector<std::vector<bool>> occ_grid(grid_width, std::vector<bool>(grid_height, false));
      
      // Marca as células ocupadas (para cada nó ocupado)
      for (auto &pt : points)
      {
        double x, y;
        bool occ;
        std::tie(x, y, occ) = pt;
        if (!occ)
          continue;
        int i = static_cast<int>(std::floor((x - min_x_layer) / resolution));
        int j = static_cast<int>(std::floor((y - min_y_layer) / resolution));
        if (i >= 0 && i < grid_width && j >= 0 && j < grid_height)
          occ_grid[i][j] = true;
      }
      
      // ====================================================
      // Calcula o "distance transform" via Dijkstra multi‐fonte
      // ====================================================
      const double INF = std::numeric_limits<double>::infinity();
      std::vector<std::vector<double>> dist(grid_width, std::vector<double>(grid_height, INF));
      // Além da distância, armazenamos qual a célula ocupada (na grade) que foi a fonte
      std::vector<std::vector<std::pair<int,int>>> nearest_occ(
          grid_width, std::vector<std::pair<int,int>>(grid_height, {-1, -1})
      );
      
      std::priority_queue<GridCell, std::vector<GridCell>, CompareGridCell> pq;
      
      // Inicializa a fila com as células ocupadas (distância zero) e registra sua posição como fonte
      for (int i = 0; i < grid_width; i++) {
        for (int j = 0; j < grid_height; j++) {
          if (occ_grid[i][j]) {
            dist[i][j] = 0.0;
            nearest_occ[i][j] = {i, j};
            pq.push({i, j, 0.0});
          }
        }
      }
      
      // Vizinhança 8-conectada: custo de "resolução" (adjacente) ou √2 * resolução (diagonal)
      std::vector<std::pair<int,int>> neighbors = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
      };
      
      while (!pq.empty()) {
        GridCell cell = pq.top();
        pq.pop();
        int i = cell.i, j = cell.j;
        if (cell.dist > dist[i][j])
          continue;
        for (auto &nb : neighbors) {
          int ni = i + nb.first;
          int nj = j + nb.second;
          if (ni < 0 || ni >= grid_width || nj < 0 || nj >= grid_height)
            continue;
          double cost = (std::abs(nb.first) + std::abs(nb.second) == 2)
                          ? resolution * std::sqrt(2.0)
                          : resolution;
          double new_dist = dist[i][j] + cost;
          if (new_dist < dist[ni][nj]) {
            dist[ni][nj] = new_dist;
            // Propaga a fonte (a célula ocupada original)
            nearest_occ[ni][nj] = nearest_occ[i][j];
            pq.push({ni, nj, new_dist});
          }
        }
      }
      
      // ====================================================
      // Seleciona as células cuja distância seja aproximadamente territorial_distance_
      // e gera a pose (posição + orientação) para o drone
      // ====================================================

      double tol = resolution / 2.0;  // tolerância
      for (int i = 0; i < grid_width; i++) {
        for (int j = 0; j < grid_height; j++) {
          if (std::abs(dist[i][j] - territorial_distance_) < tol) {

            if ((i + j) % (100 / static_cast<int>(percent_remaining_goals_)) == 0){
            // Calcula o centro da célula na camada atual
            double cell_center_x = min_x_layer + i * resolution + resolution / 2.0;
            double cell_center_y = min_y_layer + j * resolution + resolution / 2.0;
            double cell_center_z = layer * slice_thickness_ + slice_thickness_ / 2.0;
            geometry_msgs::msg::Point p;
            p.x = cell_center_x;
            p.y = cell_center_y;
            p.z = cell_center_z;
            
            // Verifica se há uma interseção (por exemplo, com o teto) – se não houver, consideramos o ponto
            octomap::point3d end_ray;
            bool hit = tree->castRay(octomap::point3d(p.x, p.y, p.z),
                                       octomap::point3d(0, 0, 1),
                                       end_ray, true, -1);
            if (!hit) {
              // Obtém a célula ocupada (na grade) que serviu de fonte
              auto occ_idx = nearest_occ[i][j];
              double occ_x = min_x_layer + occ_idx.first * resolution + resolution / 2.0;
              double occ_y = min_y_layer + occ_idx.second * resolution + resolution / 2.0;
              
              // Calcula o ângulo yaw para apontar do ponto candidato para o voxel ocupado
              double yaw = std::atan2(occ_y - p.y, occ_x - p.x);
              tf2::Quaternion q;
              q.setRPY(0, 0, yaw);
              q.normalize();
              
              geometry_msgs::msg::Pose pose;
              pose.position = p;
              pose.orientation.x = q.x();
              pose.orientation.y = q.y();
              pose.orientation.z = q.z();
              pose.orientation.w = q.w();
              
              pose_array.poses.push_back(pose);

            }
            }
          }
        }
      }
    }

    // Publica a lista de poses (posição + orientação) para o drone
    pose_pub_->publish(pose_array);
    
    // (Opcional) Se desejar manter a publicação dos marcadores visuais (por exemplo, POINTS), basta ajustar aqui.
    // ...
  }
  
  // Parâmetros configuráveis
  double slice_thickness_;
  double layer_margin_;
  double territorial_distance_;
  double percent_remaining_goals_;
  
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerritorialWatersNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

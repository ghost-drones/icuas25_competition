#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <queue>
#include <vector>
#include <unordered_map>
#include <tuple>
#include <limits>
#include <cmath>
#include <memory>
#include <algorithm>

/// Estrutura para células da grade (usada no Dijkstra)
struct GridCell {
  int i, j;
  double dist;
};

/// Comparador para a fila de prioridade (min-heap)
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
    // - slice_thickness: espessura do recorte em z (em metros)
    // - layer_margin: quanto aumentar os limites (bounding box) da camada
    // - territorial_distance: distância utilizada no cálculo de territorial waters
    slice_thickness_     = 1.0;  // Ex.: 1 metro por camada
    layer_margin_        = 5.0;  // Ex.: amplia os limites em 5 metros
    territorial_distance_= 3.0;  // Ex.: territorial waters a 3 metros da costa

    // Subscreve o tópico /octomap
    subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap", 1,
      std::bind(&TerritorialWatersNode::octomapCallback, this, std::placeholders::_1)
    );
    // Publica os marcadores para visualização
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("network_markers", 1);
    octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/octomap_with_ground", 1);
    
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

      // Cálculo do índice da camada usando slice_thickness_
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

    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;

    // Processa cada camada (fatia em z)
    for (auto &layer_pair : layers)
    {
      int layer = layer_pair.first;
      auto &points = layer_pair.second;

      if (layer_bounds.find(layer) == layer_bounds.end())
        continue;
      
      double min_x, max_x, min_y, max_y;
      std::tie(min_x, max_x, min_y, max_y) = layer_bounds[layer];
      
      // Aumenta os limites da camada de acordo com o parâmetro layer_margin_
      min_x -= layer_margin_;
      max_x += layer_margin_;
      min_y -= layer_margin_;
      max_y += layer_margin_;
      
      // Define a grade 2D com passo igual à resolução do octomap
      int grid_width  = static_cast<int>(std::ceil((max_x - min_x) / resolution));
      int grid_height = static_cast<int>(std::ceil((max_y - min_y) / resolution));
      
      // Cria uma grade de ocupação (inicialmente "livre")
      std::vector<std::vector<bool>> occ_grid(grid_width, std::vector<bool>(grid_height, false));
      
      // Marca as células ocupadas (onde há nó ocupado)
      for (auto &pt : points)
      {
        double x, y;
        bool occ;
        std::tie(x, y, occ) = pt;
        if (!occ)
          continue;
        int i = static_cast<int>(std::floor((x - min_x) / resolution));
        int j = static_cast<int>(std::floor((y - min_y) / resolution));
        if (i >= 0 && i < grid_width && j >= 0 && j < grid_height)
          occ_grid[i][j] = true;
      }
  
      
      // ===============================
      // Calcula o "distance transform" utilizando Dijkstra multi‐fonte
      // ===============================
      const double INF = std::numeric_limits<double>::infinity();
      std::vector<std::vector<double>> dist(grid_width, std::vector<double>(grid_height, INF));
      std::priority_queue<GridCell, std::vector<GridCell>, CompareGridCell> pq;
      
      // Inicializa a fila com as células ocupadas (distância zero)
      for (int i = 0; i < grid_width; i++) {
        for (int j = 0; j < grid_height; j++) {
          if (occ_grid[i][j]) {
            dist[i][j] = 0.0;
            pq.push({i, j, 0.0});
          }
        }
      }
      
      // Vizinhança 8-conectada: custo de 1 ou √2
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
          double cost = (std::abs(nb.first) + std::abs(nb.second) == 2) ? resolution * std::sqrt(2.0) : resolution;
          double new_dist = dist[i][j] + cost;
          if (new_dist < dist[ni][nj]) {
            dist[ni][nj] = new_dist;
            pq.push({ni, nj, new_dist});
          }
        }
      }
      
      // ===============================
      // Seleciona as células cuja distância seja aproximadamente territorial_distance_
      // ===============================
      std::vector<geometry_msgs::msg::Point> marker_points;
      double tol = resolution / 2.0;  // tolerância
      for (int i = 0; i < grid_width; i++) {
        for (int j = 0; j < grid_height; j++) {

          // Se a célula tem distância aproximadamente igual a territorial_distance_
          if (std::abs(dist[i][j] - territorial_distance_) < tol) {
            geometry_msgs::msg::Point p;
            p.x = min_x + i * resolution + resolution / 2.0;
            p.y = min_y + j * resolution + resolution / 2.0;
            p.z = layer * slice_thickness_ + slice_thickness_ / 2.0;

            octomap::point3d end_ray;
            bool hit = tree->castRay(octomap::point3d(p.x, p.y, p.z), octomap::point3d(0,0,1), end_ray, true, -1);
            
            if (!hit){
              marker_points.push_back(p);
            }
          }
        }
      }
      
      // Marcador para os pontos de territorial waters (apenas nas regiões válidas)
      visualization_msgs::msg::Marker points_marker;
      points_marker.header.frame_id = "world";
      points_marker.header.stamp = this->get_clock()->now();
      points_marker.ns = "drone_positions";
      points_marker.id = marker_id++;
      points_marker.type = visualization_msgs::msg::Marker::POINTS;
      points_marker.action = visualization_msgs::msg::Marker::ADD;
      points_marker.scale.x = 0.1;
      points_marker.scale.y = 0.1;
      points_marker.color.r = 0.0;
      points_marker.color.g = 0.0;
      points_marker.color.b = 1.0;
      points_marker.color.a = 1.0;
      points_marker.points = marker_points;
      
      marker_array.markers.push_back(points_marker);
    }

    octomap_msgs::msg::Octomap new_msg;
    octomap_msgs::fullMapToMsg(*tree, new_msg);
    new_msg.header.frame_id = msg->header.frame_id;
    new_msg.header.stamp = this->now();

    // Publica o Octomap modificado
    octomap_pub_->publish(new_msg);

    // Publica os marcadores (os pontos de territorial waters filtrados)
    marker_pub_->publish(marker_array);
  }
  
  // Parâmetros configuráveis
  double slice_thickness_;
  double layer_margin_;
  double territorial_distance_;
  
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerritorialWatersNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
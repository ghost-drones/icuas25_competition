#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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
    //   territorial_distance: distância desejada do obstáculo para a pose
    //   min_pose_distance: espaçamento mínimo entre poses (downsampling)
    //   tol_factor: fator multiplicativo da resolução para definir a tolerância na seleção de células candidatas
    slice_thickness_      = 2.8;   // altura da camada
    layer_margin_         = 5.0;   // margem extra para definir a grade
    territorial_distance_ = 3.0;   // distância desejada até o obstáculo
    min_pose_distance_    = 3.0;   // espaçamento mínimo entre poses
    tol_factor_           = 1.0;   // tol = tol_factor_ * resolution

    subscription_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap", 1,
      std::bind(&TerritorialWatersNode::octomapCallback, this, std::placeholders::_1)
    );

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

    // Agrupa os nós folhas por camada (fatia em z)
    std::unordered_map<int, std::vector<std::tuple<double, double, bool>>> layers;
    // Para cada camada, guarda os limites: min_x, max_x, min_y, max_y
    std::unordered_map<int, std::tuple<double, double, double, double>> layer_bounds;

    double min_x, min_y, min_z;
    tree->getMetricMin(min_x, min_y, min_z);

    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();

      // Define a camada com base na altura (slice_thickness_)
      int layer = static_cast<int>(std::floor(z / slice_thickness_));
      bool occupied = tree->isNodeOccupied(*it);
      layers[layer].push_back(std::make_tuple(x, y, occupied));
      
      // Atualiza os limites (bounding box) da camada
      if (layer_bounds.find(layer) == layer_bounds.end()) {
        layer_bounds[layer] = std::make_tuple(x, x, y, y);
      } else {
        auto &bounds = layer_bounds[layer];
        std::get<0>(bounds) = std::min(std::get<0>(bounds), x);
        std::get<1>(bounds) = std::max(std::get<1>(bounds), x);
        std::get<2>(bounds) = std::min(std::get<2>(bounds), y);
        std::get<3>(bounds) = std::max(std::get<3>(bounds), y);
      }
    }

    // Inicializa a mensagem com as poses finais
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "world";
    pose_array.header.stamp = this->get_clock()->now();

    // Define a camada inferior (geralmente o solo) e ignora-a
    int min_layer = std::numeric_limits<int>::max();
    for (const auto &layer_pair : layers) {
      min_layer = std::min(min_layer, layer_pair.first);
    }

    // Processa cada camada, exceto a inferior
    for (auto &layer_pair : layers)
    {
      int layer = layer_pair.first;
      if (layer == min_layer)
        continue;

      auto &points = layer_pair.second;
      if (layer_bounds.find(layer) == layer_bounds.end())
        continue;
      
      double min_x_layer, max_x_layer, min_y_layer, max_y_layer;
      std::tie(min_x_layer, max_x_layer, min_y_layer, max_y_layer) = layer_bounds[layer];
      
      // Expande os limites com base na margem
      min_x_layer -= layer_margin_;
      max_x_layer += layer_margin_;
      min_y_layer -= layer_margin_;
      max_y_layer += layer_margin_;
      
      // Cria a grade 2D com tamanho definido pela resolução
      int grid_width  = static_cast<int>(std::ceil((max_x_layer - min_x_layer) / resolution));
      int grid_height = static_cast<int>(std::ceil((max_y_layer - min_y_layer) / resolution));
      
      // Inicializa a grade de ocupação
      std::vector<std::vector<bool>> occ_grid(grid_width, std::vector<bool>(grid_height, false));
      
      // Marca as células ocupadas
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
      // Armazena qual célula ocupada foi a fonte para cada célula
      std::vector<std::vector<std::pair<int,int>>> nearest_occ(
          grid_width, std::vector<std::pair<int,int>>(grid_height, {-1, -1})
      );
      
      std::priority_queue<GridCell, std::vector<GridCell>, CompareGridCell> pq;
      
      // Inicializa a fila com as células ocupadas (distância zero)
      for (int i = 0; i < grid_width; i++) {
        for (int j = 0; j < grid_height; j++) {
          if (occ_grid[i][j]) {
            dist[i][j] = 0.0;
            nearest_occ[i][j] = {i, j};
            pq.push({i, j, 0.0});
          }
        }
      }
      
      // Vizinhança 8-conectada
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
            nearest_occ[ni][nj] = nearest_occ[i][j];
            pq.push({ni, nj, new_dist});
          }
        }
      }
      
      // ====================================================
      // Seleciona células com distância próxima de territorial_distance_
      // A tolerância é definida como: tol = tol_factor_ * resolution
      // ====================================================
      double tol = tol_factor_ * resolution;
      std::vector<geometry_msgs::msg::Pose> candidate_poses;
      
      for (int i = 0; i < grid_width; i++) {
        for (int j = 0; j < grid_height; j++) {
          if (std::abs(dist[i][j] - territorial_distance_) < tol) {

            // Calcula o centro da célula
            double cell_center_x = min_x_layer + i * resolution + resolution / 2.0;
            double cell_center_y = min_y_layer + j * resolution + resolution / 2.0;
            double cell_center_z = layer * slice_thickness_ + slice_thickness_ / 2.0;
            geometry_msgs::msg::Point p;
            p.x = cell_center_x;
            p.y = cell_center_y;
            p.z = cell_center_z;
            
            // (Opcional) Verifica se há um obstáculo acima (por exemplo, teto)
            octomap::point3d end_ray;
            bool hit = tree->castRay(octomap::point3d(p.x, p.y, p.z),
                                       octomap::point3d(0, 0, 1),
                                       end_ray, true, -1);
            if (!hit) {
              // Recupera a célula ocupada que serviu de fonte
              auto occ_idx = nearest_occ[i][j];
              double occ_x = min_x_layer + occ_idx.first * resolution + resolution / 2.0;
              double occ_y = min_y_layer + occ_idx.second * resolution + resolution / 2.0;
              
              // Calcula o ângulo para orientar o drone em direção ao obstáculo
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
              
              candidate_poses.push_back(pose);
            }
          }
        }
      }

      if (candidate_poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Nenhum candidato encontrado na camada %d", layer);
        continue;
      }

      // ====================================================
      // (Opcional) Ordena os candidatos por ângulo em relação ao centroide
      // Caso os candidatos formem um contorno, essa ordenação pode ajudar,
      // mas ela NÃO é indispensável para a filtragem global.
      // ====================================================
      double sum_x = 0.0, sum_y = 0.0;
      for (const auto &pose : candidate_poses) {
        sum_x += pose.position.x;
        sum_y += pose.position.y;
      }
      double centroid_x = sum_x / candidate_poses.size();
      double centroid_y = sum_y / candidate_poses.size();

      std::sort(candidate_poses.begin(), candidate_poses.end(),
                [centroid_x, centroid_y](const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b) {
                  double angle_a = std::atan2(a.position.y - centroid_y, a.position.x - centroid_x);
                  double angle_b = std::atan2(b.position.y - centroid_y, b.position.x - centroid_x);
                  return angle_a < angle_b;
                });

      // ====================================================
      // Filtragem Global: verifica cada candidato em relação a todas as poses aceitas
      // ====================================================
      std::vector<geometry_msgs::msg::Pose> filtered_poses;
      for (const auto &pose_candidate : candidate_poses) {
        bool keep = true;
        for (const auto &accepted_pose : filtered_poses) {
          double dx = pose_candidate.position.x - accepted_pose.position.x;
          double dy = pose_candidate.position.y - accepted_pose.position.y;
          double dz = pose_candidate.position.z - accepted_pose.position.z;
          double d = std::sqrt(dx * dx + dy * dy + dz * dz);
          if (d < min_pose_distance_) {
            keep = false;
            break;
          }
        }
        if (keep) {
          filtered_poses.push_back(pose_candidate);
        }
      }

      // Adiciona as poses filtradas ao array global
      for (const auto &pose : filtered_poses) {
        pose_array.poses.push_back(pose);
      }
    } // fim do processamento por camada

    // Publica todas as poses filtradas (para todas as camadas)
    pose_pub_->publish(pose_array);
  }
  
  // Parâmetros configuráveis
  double slice_thickness_;
  double layer_margin_;
  double territorial_distance_;
  double min_pose_distance_;
  double tol_factor_; // tol = tol_factor_ * resolution
  
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

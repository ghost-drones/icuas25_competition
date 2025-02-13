#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <queue>
#include <unordered_map>

struct Edge {
  int source;
  int target;
  double weight;
};

class GraphBuilder : public rclcpp::Node {
public:
  GraphBuilder()
  : Node("graph_builder"), graph_built_(false)
  {
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/drone_poses", 10, std::bind(&GraphBuilder::poseCallback, this, std::placeholders::_1));
    
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap", 10, std::bind(&GraphBuilder::octomapCallback, this, std::placeholders::_1));

    graph_pub_ = this->create_publisher<std_msgs::msg::Int32>("/graph_finished", 10);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    last_pose_array_ = msg;
    attemptGraphBuild();
  }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    last_octomap_ = msg;
    attemptGraphBuild();
  }

  void attemptGraphBuild()
  {
    if (!last_pose_array_ || !last_octomap_ || graph_built_) {
      return;
    }

    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*last_octomap_);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    if (!octree) {
      RCLCPP_ERROR(this->get_logger(), "Falha na conversão do Octomap para OcTree.");
      return;
    }

    std::vector<octomap::point3d> vertices;
    vertices.push_back(octomap::point3d(0.0f, 0.0f, 0.0f));
    for (const auto & pose : last_pose_array_->poses) {
      vertices.push_back(octomap::point3d(pose.position.x, pose.position.y, pose.position.z));
    }

    std::vector<Edge> graph_edges;
    for (size_t i = 0; i < vertices.size(); ++i) {
      for (size_t j = i + 1; j < vertices.size(); ++j) {
        double dist = (vertices[j] - vertices[i]).norm();
        if (dist < 1e-6) continue;
        
        octomap::point3d direction = vertices[j] - vertices[i];
        direction /= dist;
        octomap::point3d hit;
        bool obstacle_hit = octree->castRay(vertices[i], direction, hit, dist);
        if (!obstacle_hit) {
          graph_edges.push_back({static_cast<int>(i), static_cast<int>(j), dist});
        }
      }
    }

    computeMinDistances(vertices.size(), graph_edges);

    std_msgs::msg::Int32 msg;
    msg.data = 1;
    graph_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Grafo finalizado.");
    
    delete octree;
    graph_built_ = true;
  }

  void computeMinDistances(int num_vertices, const std::vector<Edge>& edges) {
    std::vector<std::vector<int>> adjacency_list(num_vertices);
    for (const auto& edge : edges) {
      adjacency_list[edge.source].push_back(edge.target);
      adjacency_list[edge.target].push_back(edge.source);
    }

    std::vector<int> distances(num_vertices, -1);
    std::queue<int> q;
    q.push(0);
    distances[0] = 0;

    while (!q.empty()) {
      int current = q.front(); q.pop();
      for (int neighbor : adjacency_list[current]) {
        if (distances[neighbor] == -1) {
          distances[neighbor] = distances[current] + 1;
          q.push(neighbor);
        }
      }
    }

    std::unordered_map<int, int> distance_count;
    for (int d : distances) {
      if (d > 0) {
        distance_count[d]++;
      }
    }

    for (const auto& pair : distance_count) {
      RCLCPP_INFO(this->get_logger(), "%d vértices estão a %d arestas da origem", pair.second, pair.first);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr graph_pub_;
  geometry_msgs::msg::PoseArray::SharedPtr last_pose_array_;
  octomap_msgs::msg::Octomap::SharedPtr last_octomap_;
  bool graph_built_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraphBuilder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
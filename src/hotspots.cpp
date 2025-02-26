#include <memory>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <map>
#include <sstream>
#include <cstdint>
#include <cstdlib>    // Necessário para std::getenv

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

// Inclua os headers dos seus msgs customizados
#include "icuas25_msgs/msg/waypoint_info.hpp"
#include "icuas25_msgs/msg/waypoints.hpp"

// Representa um ponto 3D
struct Vector3 {
  double x, y, z;
  Vector3(double _x=0, double _y=0, double _z=0): x(_x), y(_y), z(_z){}
  Vector3 operator+(const Vector3 &o) const { return Vector3(x+o.x, y+o.y, z+o.z); }
  Vector3 operator-(const Vector3 &o) const { return Vector3(x-o.x, y-o.y, z-o.z); }
  Vector3 operator*(double s) const { return Vector3(x*s, y*s, z*s); }
  double norm() const { return std::sqrt(x*x + y*y + z*z); }
  Vector3 normalized() const { double n = norm(); return n < 1e-6 ? Vector3() : (*this)*(1.0/n); }
};

// Estrutura para armazenar informações de um hotspot (ou subhotspot)
struct HotspotInfo {
  Vector3 center;
  int degree; // 0 para o hotspot inicial, 1 para os conectados diretamente, etc.
  std::vector<Vector3> connected; // Lista das poses conectadas por esse nó
};

// Função auxiliar para agrupar pontos (sem alterações)
std::vector<std::vector<Vector3>> clusterPoints(const std::vector<Vector3>& pts, double thr) {
  std::vector<std::vector<Vector3>> clusters; 
  std::vector<bool> vis(pts.size(), false);
  for (size_t i = 0; i < pts.size(); i++){
    if (vis[i]) continue;
    std::vector<Vector3> cl; 
    std::vector<size_t> st = {i}; 
    vis[i] = true;
    while (!st.empty()){
      size_t idx = st.back();
      st.pop_back();
      cl.push_back(pts[idx]);
      for (size_t j = 0; j < pts.size(); j++){
        if (!vis[j] && (pts[j] - pts[idx]).norm() < thr){ 
          vis[j] = true;
          st.push_back(j);
        }
      }
    }
    clusters.push_back(cl);
  }
  return clusters;
}

// Classe principal
class WifiRangeVisualizer: public rclcpp::Node {
public:
  WifiRangeVisualizer(): Node("wifi_range_visualizer") {
    // Subscrição para octomap e poses dos drones
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/ghost/octomap", 10, std::bind(&WifiRangeVisualizer::octomapCallback, this, std::placeholders::_1));
    drone_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/ghost/drone_poses", 10, std::bind(&WifiRangeVisualizer::dronePosesCallback, this, std::placeholders::_1));
    // Publicadores para os marcadores RViz e para os waypoints customizados
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/ghost/visualization_marker_array", 10);
    waypoints_pub_ = this->create_publisher<icuas25_msgs::msg::Waypoints>("/ghost/waypoints", 10);

    // Valor padrão para d0_
    d0_ = 20.0;
    // Tenta atualizar d0_ a partir da variável de ambiente "COMM_RANGE"
    const char* comm_range_env = std::getenv("COMM_RANGE");
    if (comm_range_env != nullptr) {
      try {
        d0_ = std::stod(comm_range_env);
        RCLCPP_INFO(this->get_logger(), "COMM_RANGE definido como: %.2f", d0_);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Valor inválido em COMM_RANGE. Usando default: 20.0");
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "COMM_RANGE não definido. Usando default: 20.0");
    }
  }
  
private:
  // Função para verificar LOS direta entre dois pontos (já existente)
  bool freeLOS(const Vector3 &a, const Vector3 &b, octomap::OcTree* octree, double d0) {
    double dist = (b - a).norm();
    if(dist > d0)
      return false;
    Vector3 d = b - a;
    double rl = d.norm();
    Vector3 rdir = (rl < 1e-6 ? Vector3() : d.normalized());
    octomap::point3d end;
    return !octree->castRay(octomap::point3d(a.x, a.y, a.z),
                             octomap::point3d(rdir.x, rdir.y, rdir.z),
                             end, true, rl);
  }

  // Função que verifica LOS entre dois pontos e testa pontos intermediários
  bool freeLOSWithHotspot(const Vector3 &hotspot, const Vector3 &a, const Vector3 &b, octomap::OcTree* octree, double d0) {
    if (!freeLOS(a, b, octree, d0))
      return false;
    const int num_samples = 10;
    for (int i = 1; i < num_samples; i++) {
      double t = static_cast<double>(i) / static_cast<double>(num_samples);
      Vector3 p = a + (b - a) * t;
      if (!freeLOS(hotspot, p, octree, d0))
        return false;
    }
    return true;
  }
  
  // Callback para atualizar as poses dos drones
  void dronePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg){ 
    latest_drone_poses_ = msg; 
  }
  
  // Callback principal que processa o octomap, calcula conexões e publica os waypoints e marcadores RViz
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    try {
      octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
      if(!tree) {
        RCLCPP_ERROR(this->get_logger(), "Falha ao converter octomap para árvore.");
        return;
      }
      octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
      if(!octree){ 
        RCLCPP_ERROR(this->get_logger(), "Octree dinâmico falhou.");
        delete tree; 
        return;
      }
      
      visualization_msgs::msg::MarkerArray ma;
      int mid = 0;
      
      // Cria marcadores para os alcances WiFi (esferas de referência)
      double min_x, min_y, min_z, max_x, max_y, max_z;
      octree->getMetricMin(min_x, min_y, min_z);
      octree->getMetricMax(max_x, max_y, max_z);
      double max_range = 0;
      std::vector<Vector3> corners = { Vector3(min_x, min_y, min_z), Vector3(min_x, min_y, max_z),
                                       Vector3(min_x, max_y, min_z), Vector3(min_x, max_y, max_z),
                                       Vector3(max_x, min_y, min_z), Vector3(max_x, min_y, max_z),
                                       Vector3(max_x, max_y, min_z), Vector3(max_x, max_y, max_z) };
      for(auto &c: corners)
        max_range = std::max(max_range, c.norm());
      
      // Usa o valor de d0_ definido a partir da variável de ambiente
      for(double r = d0_; r <= max_range + d0_; r += d0_) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = msg->header.frame_id;
        m.header.stamp = this->now();
        m.ns = "wifi_ranges";
        m.id = mid++;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.1;
        m.color.r = 0.8; m.color.g = 0.8; m.color.b = 0.8; m.color.a = 0.5;
        int np = 50, sd = 10;
        for(int j = 0; j <= sd; j++){
          double phi = (j * 90.0 / sd) * M_PI / 180.0;
          Vector3 d(std::cos(phi), 0, std::sin(phi)), u(0, 1, 0);
          for(int i = 0; i <= np; i++){
            double theta = -M_PI/2 + (M_PI * i) / double(np);
            Vector3 pt = (d * std::cos(theta) + u * std::sin(theta)) * r;
            geometry_msgs::msg::Point p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            m.points.push_back(p);
          }
        }
        m.lifetime = rclcpp::Duration::from_seconds(0);
        ma.markers.push_back(m);
      }
      
      // Se não houver poses dos drones, publica apenas os marcadores de alcance e encerra
      if(!latest_drone_poses_ || latest_drone_poses_->poses.empty()){
        marker_array_pub_->publish(ma);
        delete octree;
        return;
      }
      
      // PROCESSAMENTO GLOBAL DE POSES
      std::vector<Vector3> allPoses;
      std::vector<geometry_msgs::msg::Pose> allPosesFull;
      std::vector<bool> connected;
      std::vector<int> poseDegrees;
      for(auto &p: latest_drone_poses_->poses) {
        Vector3 pos(p.position.x, p.position.y, p.position.z);
        allPoses.push_back(pos);
        allPosesFull.push_back(p);
        connected.push_back(false);
        poseDegrees.push_back(-1);
      }
      
      // Armazena o índice do nó pai e se foi usado para expandir (transition point)
      std::vector<int> parent_index(allPoses.size(), -1);
      std::vector<bool> is_transition(allPoses.size(), false);
      
      // Armazena informações dos hotspots/subhotspots
      std::vector<HotspotInfo> hotspotInfos;
      
      // 1. Cria o hotspot inicial na origem e conecta poses com LOS e distância <= d0_
      Vector3 rootHotspot(0,0,0);
      HotspotInfo rootInfo;
      rootInfo.center = rootHotspot;
      rootInfo.degree = 0;
      visualization_msgs::msg::Marker initRel;
      initRel.header.frame_id = msg->header.frame_id;
      initRel.header.stamp = this->now();
      initRel.ns = "initial_relations";
      initRel.id = mid++;
      initRel.type = visualization_msgs::msg::Marker::LINE_LIST;
      initRel.action = visualization_msgs::msg::Marker::ADD;
      initRel.scale.x = 0.1;
      initRel.color.r = 1.0; initRel.color.g = 1.0; initRel.color.b = 0.0; initRel.color.a = 1.0;
      
      for (size_t i = 0; i < allPoses.size(); i++){
        if ((allPoses[i] - rootHotspot).norm() <= d0_ && 
            freeLOS(rootHotspot, allPoses[i], octree, d0_)) {
          connected[i] = true;
          poseDegrees[i] = 1;
          parent_index[i] = -1;
          rootInfo.connected.push_back(allPoses[i]);
          geometry_msgs::msg::Point p1, p2;
          p1.x = rootHotspot.x; p1.y = rootHotspot.y; p1.z = rootHotspot.z;
          p2.x = allPoses[i].x; p2.y = allPoses[i].y; p2.z = allPoses[i].z;
          initRel.points.push_back(p1);
          initRel.points.push_back(p2);
        }
      }
      if(!initRel.points.empty()){
        initRel.lifetime = rclcpp::Duration::from_seconds(0);
        ma.markers.push_back(initRel);
      }
      hotspotInfos.push_back(rootInfo);
      
      // 2. Expansão iterativa de subhotspots (transition points)
      bool progress = true;
      while(progress) {
        progress = false;
        int candidateIndex = -1;
        int bestCount = -1;
        int minDegree = std::numeric_limits<int>::max();
        for (size_t i = 0; i < allPoses.size(); i++){
          if (connected[i]) {
            int cnt = 0;
            for (size_t j = 0; j < allPoses.size(); j++){
              if (!connected[j] && ((allPoses[j] - allPoses[i]).norm() < d0_) &&
                  freeLOS(allPoses[i], allPoses[j], octree, d0_))
                cnt++;
            }
            if (cnt > 0 && poseDegrees[i] < minDegree)
              minDegree = poseDegrees[i];
          }
        }
        for (size_t i = 0; i < allPoses.size(); i++){
          if (connected[i] && poseDegrees[i] == minDegree) {
            int cnt = 0;
            for (size_t j = 0; j < allPoses.size(); j++){
              if (!connected[j] && ((allPoses[j] - allPoses[i]).norm() < d0_) &&
                  freeLOS(allPoses[i], allPoses[j], octree, d0_))
                cnt++;
            }
            if (cnt > bestCount){
              bestCount = cnt;
              candidateIndex = i;
            }
          }
        }
        
        if(candidateIndex == -1 || bestCount == 0)
          break;
        
        Vector3 subhot = allPoses[candidateIndex];
        is_transition[candidateIndex] = true;
        std::vector<Vector3> newConnected;
        
        visualization_msgs::msg::Marker subRel;
        subRel.header.frame_id = msg->header.frame_id;
        subRel.header.stamp = this->now();
        subRel.ns = "sub_relations";
        subRel.id = mid++;
        subRel.type = visualization_msgs::msg::Marker::LINE_LIST;
        subRel.action = visualization_msgs::msg::Marker::ADD;
        subRel.scale.x = 0.1;
        if(poseDegrees[candidateIndex] > 1) {
          subRel.color.r = 0.0; subRel.color.g = 1.0; subRel.color.b = 1.0; subRel.color.a = 1.0;
        } else {
          subRel.color.r = 1.0; subRel.color.g = 0.0; subRel.color.b = 1.0; subRel.color.a = 1.0;
        }
        
        for (size_t j = 0; j < allPoses.size(); j++){
          if (!connected[j] && ((allPoses[j] - subhot).norm() < d0_) &&
              freeLOS(subhot, allPoses[j], octree, d0_)) {
            geometry_msgs::msg::Point p1, p2;
            p1.x = subhot.x; p1.y = subhot.y; p1.z = subhot.z;
            p2.x = allPoses[j].x; p2.y = allPoses[j].y; p2.z = allPoses[j].z;
            subRel.points.push_back(p1);
            subRel.points.push_back(p2);
            connected[j] = true;
            poseDegrees[j] = poseDegrees[candidateIndex] + 1;
            parent_index[j] = candidateIndex;
            newConnected.push_back(allPoses[j]);
            progress = true;
          }
        }
        if(!subRel.points.empty()){
          subRel.lifetime = rclcpp::Duration::from_seconds(0);
          ma.markers.push_back(subRel);
        }
        if(!newConnected.empty()){
          HotspotInfo info;
          info.center = subhot;
          info.degree = poseDegrees[candidateIndex];
          info.connected = newConnected;
          hotspotInfos.push_back(info);
        }
      }
      
      // 3. Calcula a "order" de cada pose (número de transition points na cadeia até a origem)
      std::vector<int> orders(allPoses.size(), 0);
      for (size_t i = 0; i < allPoses.size(); i++){
        int order_count = 0;
        int cur = parent_index[i];
        while(cur != -1){
          if(is_transition[cur])
            order_count++;
          cur = parent_index[cur];
        }
        orders[i] = order_count;
      }
      
      // 4. Determina o cluster de cada pose usando a mesma lógica da visualização.
      // O cluster_id dos hotspots é atribuído via trans_cluster_map e para os demais é herdado do hotspot pai.
      std::map<int, int> trans_cluster_map;
      int next_cluster_id = 1;
      std::vector<int> cluster_ids(allPoses.size(), 0); // 0 para o cluster base
      for (size_t i = 0; i < allPoses.size(); i++){
        if(is_transition[i]){
          trans_cluster_map[i] = next_cluster_id;
          cluster_ids[i] = next_cluster_id;
          next_cluster_id++;
        }
      }
      for (size_t i = 0; i < allPoses.size(); i++){
        if(!is_transition[i]){
          int best_candidate = -1;
          int best_degree = -1;
          for (size_t j = 0; j < allPoses.size(); j++){
            if(is_transition[j] && (poseDegrees[j] < poseDegrees[i])) {
              if(freeLOS(allPoses[j], allPoses[i], octree, d0_)){
                if(poseDegrees[j] > best_degree){
                  best_degree = poseDegrees[j];
                  best_candidate = j;
                }
              }
            }
          }
          if(best_candidate != -1)
            cluster_ids[i] = trans_cluster_map[best_candidate];
          else
            cluster_ids[i] = 0;
        }
      }
      
      // 5. Determina, para cada pose, a lista de waypoints com LOS dentro do mesmo cluster.
      std::vector<std::vector<int64_t>> cluster_los(allPoses.size());
      std::map<int, Vector3> cluster_hotspots;
      cluster_hotspots[0] = rootHotspot;
      for (size_t i = 0; i < allPoses.size(); i++){
        if(is_transition[i] && cluster_ids[i] != 0) {
           cluster_hotspots[cluster_ids[i]] = allPoses[i];
        }
      }
      for (size_t i = 0; i < allPoses.size(); i++){
        for (size_t j = 0; j < allPoses.size(); j++){
          if(i == j)
            continue;
          if(cluster_ids[i] == cluster_ids[j]){
            Vector3 hotspot = cluster_hotspots[cluster_ids[i]];
            if(freeLOSWithHotspot(hotspot, allPoses[i], allPoses[j], octree, d0_)){
              cluster_los[i].push_back(static_cast<int64_t>(j));
            }
          }
        }
      }
      
      // 6. Preenche e publica a mensagem Waypoints.msg.
      icuas25_msgs::msg::Waypoints waypoints_msg;
      for (size_t i = 0; i < allPoses.size(); i++){
        icuas25_msgs::msg::WaypointInfo wp;
        wp.id = static_cast<int64_t>(i);
        wp.cluster_id = static_cast<int64_t>(cluster_ids[i]);
        wp.prev_cluster_id = (parent_index[i] != -1) ? static_cast<int64_t>(cluster_ids[parent_index[i]]) : 0;
        wp.order = static_cast<int64_t>(orders[i]);
        wp.pose = allPosesFull[i];
        wp.cluster_los = cluster_los[i];
        wp.transition_point = is_transition[i];
        waypoints_msg.waypoints.push_back(wp);
      }
      waypoints_pub_->publish(waypoints_msg);
      
      // 7. Cria marcadores do tipo TEXT_VIEW_FACING para exibir o ID de cada waypoint em RViz.
      for (size_t i = 0; i < allPoses.size(); i++){
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = msg->header.frame_id;
        text_marker.header.stamp = this->now();
        text_marker.ns = "waypoint_ids";
        text_marker.id = mid++; // Incrementa id para não conflitar com outros marcadores
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose = allPosesFull[i];
        text_marker.pose.position.z += 0.5; // desloca 0.5m para cima
        text_marker.scale.z = 0.5; // tamanho do texto
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = std::to_string(i);
        text_marker.lifetime = rclcpp::Duration::from_seconds(0);
        ma.markers.push_back(text_marker);
      }
      
      // 8. Cria marcadores do tipo TEXT_VIEW_FACING para exibir o Cluster ID nos hotspots dos clusters.
      for (const auto& pair : cluster_hotspots) {
         int cluster_id = pair.first;
         if(cluster_id == 0)
           continue;
         Vector3 hotspot_pose = pair.second;
         visualization_msgs::msg::Marker cluster_text_marker;
         cluster_text_marker.header.frame_id = msg->header.frame_id;
         cluster_text_marker.header.stamp = this->now();
         cluster_text_marker.ns = "cluster_ids";
         cluster_text_marker.id = mid++;
         cluster_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
         cluster_text_marker.action = visualization_msgs::msg::Marker::ADD;
         geometry_msgs::msg::Pose cluster_pose;
         cluster_pose.position.x = hotspot_pose.x;
         cluster_pose.position.y = hotspot_pose.y;
         cluster_pose.position.z = hotspot_pose.z + 1.0; // desloca 1.0m para cima
         cluster_pose.orientation.w = 1.0;
         cluster_text_marker.pose = cluster_pose;
         cluster_text_marker.scale.z = 0.5;
         cluster_text_marker.color.r = 0.0;
         cluster_text_marker.color.g = 1.0;
         cluster_text_marker.color.b = 0.0;
         cluster_text_marker.color.a = 1.0;
         cluster_text_marker.text = "Cluster " + std::to_string(cluster_id);
         cluster_text_marker.lifetime = rclcpp::Duration::from_seconds(0);
         ma.markers.push_back(cluster_text_marker);
      }
      
      // Publica os marcadores RViz
      marker_array_pub_->publish(ma);
      delete octree;
    }
    catch(const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exceção na função octomapCallback: %s", e.what());
    }
  }
  
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drone_poses_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Publisher<icuas25_msgs::msg::Waypoints>::SharedPtr waypoints_pub_;
  geometry_msgs::msg::PoseArray::SharedPtr latest_drone_poses_;
  
  // Variável que define a distância de comunicação (d0) lida da variável de ambiente "COMM_RANGE"
  double d0_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WifiRangeVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

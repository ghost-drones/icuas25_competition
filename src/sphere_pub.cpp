#include <memory>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <map>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <octomap/octomap.h>

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
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap", 10, std::bind(&WifiRangeVisualizer::octomapCallback, this, std::placeholders::_1));
    drone_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/drone_poses", 10, std::bind(&WifiRangeVisualizer::dronePosesCallback, this, std::placeholders::_1));
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "visualization_marker_array", 10);
    hotspot_info_pub_ = this->create_publisher<std_msgs::msg::String>("hotspot_info", 10);
  }
  
private:
  void dronePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg){ 
    latest_drone_poses_ = msg; 
  }
  
  // Função que primeiro descarta conexões cujo distância é maior que d0,
  // e somente se (b - a).norm() <= d0 realiza o castRay para confirmar a linha de visão.
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
  
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    // Converte a mensagem em octree
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
    if(!tree)
      return;
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    if(!octree){ 
      delete tree; 
      return;
    }
    
    visualization_msgs::msg::MarkerArray ma;
    int mid = 0;
    
    // Marcadores dos alcances WiFi (esferas de referência)
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
    const double d0 = 20.0;
    for(double r = d0; r <= max_range + d0; r += d0) {
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
    
    if(!latest_drone_poses_ || latest_drone_poses_->poses.empty()){
      marker_array_pub_->publish(ma);
      delete octree;
      return;
    }
    
    // ****************************
    // PROCESSAMENTO GLOBAL DE POSES
    // ****************************
    // Armazena todas as poses e inicializa "connected" e "poseDegrees"
    std::vector<Vector3> allPoses;
    std::vector<bool> connected;
    std::vector<int> poseDegrees; // -1 indica não conectado
    for(auto &p: latest_drone_poses_->poses) {
      Vector3 pos(p.position.x, p.position.y, p.position.z);
      allPoses.push_back(pos);
      connected.push_back(false);
      poseDegrees.push_back(-1);
    }
    
    // Vetor para armazenar as informações dos hotspots/subhotspots
    std::vector<HotspotInfo> hotspotInfos;
    
    // 1. Cria o hotspot inicial na origem (0,0,0) e conecta todas as poses que estejam a <= d0 e com LOS
    Vector3 rootHotspot(0,0,0);
    HotspotInfo rootInfo;
    rootInfo.center = rootHotspot;
    rootInfo.degree = 0; // raiz
    visualization_msgs::msg::Marker initRel;
    initRel.header.frame_id = msg->header.frame_id;
    initRel.header.stamp = this->now();
    initRel.ns = "initial_relations";
    initRel.id = mid++;
    initRel.type = visualization_msgs::msg::Marker::LINE_LIST;
    initRel.action = visualization_msgs::msg::Marker::ADD;
    initRel.scale.x = 0.1;
    // Cor para a conexão inicial (por exemplo, amarelo)
    initRel.color.r = 1.0; initRel.color.g = 1.0; initRel.color.b = 0.0; initRel.color.a = 1.0;
    
    for (size_t i = 0; i < allPoses.size(); i++){
      if ((allPoses[i] - rootHotspot).norm() <= d0 && 
          freeLOS(rootHotspot, allPoses[i], octree, d0)) {
        connected[i] = true;
        poseDegrees[i] = 1; // Conexões diretas da raiz terão grau 1
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
    
    // 2. Expansão iterativa de subhotspots:
    //    Prioriza a expansão a partir dos nós com menor grau.
    bool progress = true;
    while(progress) {
      progress = false;
      int candidateIndex = -1;
      int bestCount = -1;
      int minDegree = std::numeric_limits<int>::max();
      // Primeiro, determina o menor grau entre os nós conectados que podem conectar outros
      for (size_t i = 0; i < allPoses.size(); i++){
        if (connected[i]) {
          int cnt = 0;
          for (size_t j = 0; j < allPoses.size(); j++){
            if (!connected[j] && ((allPoses[j] - allPoses[i]).norm() < d0) &&
                freeLOS(allPoses[i], allPoses[j], octree, d0))
              cnt++;
          }
          if (cnt > 0 && poseDegrees[i] < minDegree)
            minDegree = poseDegrees[i];
        }
      }
      // Entre os nós com o grau mínimo, escolhe o que maximiza as conexões possíveis
      for (size_t i = 0; i < allPoses.size(); i++){
        if (connected[i] && poseDegrees[i] == minDegree) {
          int cnt = 0;
          for (size_t j = 0; j < allPoses.size(); j++){
            if (!connected[j] && ((allPoses[j] - allPoses[i]).norm() < d0) &&
                freeLOS(allPoses[i], allPoses[j], octree, d0))
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
      std::vector<Vector3> newConnected; // novas conexões deste subhotspot
      
      visualization_msgs::msg::Marker subRel;
      subRel.header.frame_id = msg->header.frame_id;
      subRel.header.stamp = this->now();
      subRel.ns = "sub_relations";
      subRel.id = mid++;
      subRel.type = visualization_msgs::msg::Marker::LINE_LIST;
      subRel.action = visualization_msgs::msg::Marker::ADD;
      subRel.scale.x = 0.1;
      // Se o grau do candidato for > 1, usa cor ciano; caso contrário, magenta.
      if(poseDegrees[candidateIndex] > 1) {
        subRel.color.r = 0.0; subRel.color.g = 1.0; subRel.color.b = 1.0; subRel.color.a = 1.0;
      } else {
        subRel.color.r = 1.0; subRel.color.g = 0.0; subRel.color.b = 1.0; subRel.color.a = 1.0;
      }
      
      for (size_t j = 0; j < allPoses.size(); j++){
        if (!connected[j] && ((allPoses[j] - subhot).norm() < d0) &&
            freeLOS(subhot, allPoses[j], octree, d0)) {
          geometry_msgs::msg::Point p1, p2;
          p1.x = subhot.x; p1.y = subhot.y; p1.z = subhot.z;
          p2.x = allPoses[j].x; p2.y = allPoses[j].y; p2.z = allPoses[j].z;
          subRel.points.push_back(p1);
          subRel.points.push_back(p2);
          connected[j] = true;
          poseDegrees[j] = poseDegrees[candidateIndex] + 1;
          newConnected.push_back(allPoses[j]);
          progress = true;
        }
      }
      if(!subRel.points.empty()){
        subRel.lifetime = rclcpp::Duration::from_seconds(0);
        ma.markers.push_back(subRel);
      }
      // Registra este subhotspot se houve novas conexões
      if(!newConnected.empty()){
        HotspotInfo info;
        info.center = subhot;
        info.degree = poseDegrees[candidateIndex];
        info.connected = newConnected;
        hotspotInfos.push_back(info);
      }
    }
    
    // 3. Publica uma mensagem com as informações dos hotspots/subhotspots
    std::stringstream ss;
    for (size_t i = 0; i < hotspotInfos.size(); i++){
      ss << "Hotspot " << i << " (center: " << hotspotInfos[i].center.x << ", "
         << hotspotInfos[i].center.y << ", " << hotspotInfos[i].center.z 
         << ", degree: " << hotspotInfos[i].degree << "): Connected poses: ";
      for (auto &pt : hotspotInfos[i].connected){
        ss << "(" << pt.x << ", " << pt.y << ", " << pt.z << ") ";
      }
      ss << "\n";
    }
    std_msgs::msg::String hotspotMsg;
    hotspotMsg.data = ss.str();
    hotspot_info_pub_->publish(hotspotMsg);
    
    // 4. Publica os markers e libera a octree
    marker_array_pub_->publish(ma);
    delete octree;
  }
  
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drone_poses_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hotspot_info_pub_;
  geometry_msgs::msg::PoseArray::SharedPtr latest_drone_poses_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WifiRangeVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

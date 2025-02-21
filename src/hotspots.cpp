#include <memory>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <map>
#include <sstream>
#include <cstdint>

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
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/ghost/octomap", 10, std::bind(&WifiRangeVisualizer::octomapCallback, this, std::placeholders::_1));
    drone_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/ghost/drone_poses", 10, std::bind(&WifiRangeVisualizer::dronePosesCallback, this, std::placeholders::_1));
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/ghost/visualization_marker_array", 10);
    waypoints_pub_ = this->create_publisher<icuas25_msgs::msg::Waypoints>("/ghost/waypoints", 10);
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

  // NOVA FUNÇÃO: Verifica LOS entre dois pontos e, adicionalmente, testa pontos intermediários
  // para garantir que cada ponto no caminho possui LOS com o hotspot associado ao cluster.
  bool freeLOSWithHotspot(const Vector3 &hotspot, const Vector3 &a, const Vector3 &b, octomap::OcTree* octree, double d0) {
    // Primeiro, verifica a LOS direta entre os dois pontos.
    if (!freeLOS(a, b, octree, d0))
      return false;
    // Define o número de pontos intermediários a serem amostrados.
    const int num_samples = 10;
    // Testa pontos intermediários (excluindo os extremos, que já foram verificados)
    for (int i = 1; i < num_samples; i++) {
      double t = static_cast<double>(i) / static_cast<double>(num_samples);
      Vector3 p = a + (b - a) * t;
      if (!freeLOS(hotspot, p, octree, d0))
        return false;
    }
    return true;
  }
  
  void dronePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg){ 
    latest_drone_poses_ = msg; 
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
    std::vector<geometry_msgs::msg::Pose> allPosesFull; // Armazena a pose completa (posição + orientação)
    std::vector<bool> connected;
    std::vector<int> poseDegrees; // -1 indica não conectado
    for(auto &p: latest_drone_poses_->poses) {
      Vector3 pos(p.position.x, p.position.y, p.position.z);
      allPoses.push_back(pos);
      allPosesFull.push_back(p); // Usa a orientação conforme recebido
      connected.push_back(false);
      poseDegrees.push_back(-1);
    }
    
    // Vetores para armazenar o nó "pai" de cada pose e marcar se ela foi usada para expandir (transition point)
    std::vector<int> parent_index(allPoses.size(), -1);
    std::vector<bool> is_transition(allPoses.size(), false);
    
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
        // Conexão direta da raiz: não há pai (representado por -1)
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
      // Marca o nó candidato como transition point (hotspot) para iniciar um cluster
      is_transition[candidateIndex] = true;
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
          // Registra o nó pai de j (ligação com o transition point)
          parent_index[j] = candidateIndex;
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
    
    // ************* CÁLCULOS PARA CLUSTERIZAÇÃO E WAYPOINTS *************
    // 1. Calcula a "order" de cada pose (número de transition points na cadeia até a origem)
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
    
    // 2. Determina o cluster de cada pose.
    // A ideia: se uma pose tem linha-de-visada com um ou mais transition points (hotspots)
    // que possuem grau menor que ela, ela se associa ao que tem o maior grau (prioridade).
    std::map<int, int> trans_cluster_map; // mapeia índice do transition point para cluster_id
    int next_cluster_id = 1;
    std::vector<int> cluster_ids(allPoses.size(), 0); // 0 indica cluster base
    // Primeiro, atribui um novo cluster_id para cada transition point
    for (size_t i = 0; i < allPoses.size(); i++){
      if(is_transition[i]){
        trans_cluster_map[i] = next_cluster_id;
        cluster_ids[i] = next_cluster_id;
        next_cluster_id++;
      }
    }
    // Para as poses que não são transition points, procura entre os transition points com LOS
    // e com grau menor que o da pose, aquele com o maior grau.
    for (size_t i = 0; i < allPoses.size(); i++){
      if(!is_transition[i]){
        int best_candidate = -1;
        int best_degree = -1;
        for (size_t j = 0; j < allPoses.size(); j++){
          if(is_transition[j] && (poseDegrees[j] < poseDegrees[i])) {
            if(freeLOS(allPoses[j], allPoses[i], octree, d0)){
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
          cluster_ids[i] = 0; // se não houver, associa ao cluster base
      }
    }
    
    // 3. Para cada pose, determina a lista de waypoints (ids) com os quais possui LOS dentro do mesmo cluster.
    // Aqui, além de verificar a LOS entre os pontos, amostramos pontos ao longo do caminho e
    // garantimos que estes pontos também tenham LOS com o hotspot do cluster.
    std::vector<std::vector<int64_t>> cluster_los(allPoses.size());
    // Cria um mapeamento de cluster para hotspot: para o cluster base usa-se a raiz, e para os demais,
    // o hotspot é o transition point que originou o cluster.
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
          if(freeLOSWithHotspot(hotspot, allPoses[i], allPoses[j], octree, d0)){
            cluster_los[i].push_back(static_cast<int64_t>(j));
          }
        }
      }
    }
    
    // 4. Preenche e publica a mensagem Waypoints.msg com os WaypointInfo.msg de todas as poses.
    //    Aqui, utilizamos a orientação que veio na mensagem original, garantindo a consistência com os marcadores.
    icuas25_msgs::msg::Waypoints waypoints_msg;
    for (size_t i = 0; i < allPoses.size(); i++){
      icuas25_msgs::msg::WaypointInfo wp;
      wp.id = static_cast<int64_t>(i);
      wp.cluster_id = static_cast<int64_t>(cluster_ids[i]);
      // Novo campo: atribui prev_cluster_id com base no nó pai
      if (parent_index[i] != -1) {
        wp.prev_cluster_id = static_cast<int64_t>(cluster_ids[parent_index[i]]);
      } else {
        wp.prev_cluster_id = 0;
      }
      wp.order = static_cast<int64_t>(orders[i]);
      wp.pose = allPosesFull[i];  // Usa a pose completa, com orientação
      wp.cluster_los = cluster_los[i];
      wp.transition_point = is_transition[i];
      waypoints_msg.waypoints.push_back(wp);
    }
    waypoints_pub_->publish(waypoints_msg);
    
    // 5. Publica os markers e libera a octree
    marker_array_pub_->publish(ma);
    delete octree;
  }
  
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drone_poses_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Publisher<icuas25_msgs::msg::Waypoints>::SharedPtr waypoints_pub_;
  geometry_msgs::msg::PoseArray::SharedPtr latest_drone_poses_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WifiRangeVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

#include <memory>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <octomap/octomap.h>

struct Vector3 {
  double x, y, z;
  Vector3(double _x=0, double _y=0, double _z=0): x(_x), y(_y), z(_z){}
  Vector3 operator+(const Vector3 &o) const { return Vector3(x+o.x, y+o.y, z+o.z); }
  Vector3 operator-(const Vector3 &o) const { return Vector3(x-o.x, y-o.y, z-o.z); }
  Vector3 operator*(double s) const { return Vector3(x*s, y*s, z*s); }
  double norm() const { return std::sqrt(x*x+y*y+z*z); }
  Vector3 normalized() const { double n = norm(); return n<1e-6?Vector3():(*this)*(1.0/n); }
};

struct Hotspot { 
  Vector3 center; 
  int cluster_size, sphere_index, connection_count = 0; 
};

std::vector<std::vector<Vector3>> clusterPoints(const std::vector<Vector3>& pts, double thr) {
  std::vector<std::vector<Vector3>> clusters; std::vector<bool> vis(pts.size(), false);
  for(size_t i=0;i<pts.size();i++){
    if(vis[i]) continue;
    std::vector<Vector3> cl; std::vector<size_t> st = {i}; vis[i]=true;
    while(!st.empty()){
      size_t idx = st.back(); st.pop_back(); cl.push_back(pts[idx]);
      for(size_t j=0;j<pts.size();j++){
        if(!vis[j] && (pts[j]-pts[idx]).norm()<thr){ vis[j]=true; st.push_back(j); }
      }
    }
    clusters.push_back(cl);
  }
  return clusters;
}

class WifiRangeVisualizer: public rclcpp::Node {
public:
  WifiRangeVisualizer(): Node("wifi_range_visualizer") {
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap",10,
      std::bind(&WifiRangeVisualizer::octomapCallback,this,std::placeholders::_1));
    drone_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/drone_poses",10,
      std::bind(&WifiRangeVisualizer::dronePosesCallback,this,std::placeholders::_1));
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array",10);
  }
private:
  void dronePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg){ latest_drone_poses_ = msg; }

  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    // Converte a mensagem em octree
    octomap::AbstractOcTree* tree = octomap_msgs::fullMsgToMap(*msg);
    if(!tree)return;
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    if(!octree){ delete tree; return; }
    
    visualization_msgs::msg::MarkerArray ma; int mid=0;
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree->getMetricMin(min_x,min_y,min_z); octree->getMetricMax(max_x,max_y,max_z);
    double max_range=0; 
    std::vector<Vector3> corners = { Vector3(min_x,min_y,min_z),Vector3(min_x,min_y,max_z),
                                     Vector3(min_x,max_y,min_z),Vector3(min_x,max_y,max_z),
                                     Vector3(max_x,min_y,min_z),Vector3(max_x,min_y,max_z),
                                     Vector3(max_x,max_y,min_z),Vector3(max_x,max_y,max_z) };
    for(auto &c: corners) max_range = std::max(max_range, c.norm());
    double d0 = 20.0;
    // Wifi range markers
    for(double r = d0; r<=max_range+d0; r+=d0){
      visualization_msgs::msg::Marker m;
      m.header.frame_id = msg->header.frame_id; m.header.stamp = this->now();
      m.ns="wifi_ranges"; m.id = mid++; m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.action = visualization_msgs::msg::Marker::ADD; m.pose.orientation.w=1.0;
      m.scale.x = 0.1; m.color.r=0.8; m.color.g=0.8; m.color.b=0.8; m.color.a=0.5;
      int np=50, sd=10;
      for(int j=0;j<=sd;j++){
        double phi = (j*90.0/sd)*M_PI/180.0;
        Vector3 d(std::cos(phi),0,std::sin(phi)), u(0,1,0);
        for(int i=0;i<=np;i++){
          double theta = -M_PI/2 + (M_PI*i)/double(np);
          Vector3 pt = (d*std::cos(theta) + u*std::sin(theta))*r;
          geometry_msgs::msg::Point p; p.x=pt.x; p.y=pt.y; p.z=pt.z; m.points.push_back(p);
        }
      }
      m.lifetime = rclcpp::Duration::from_seconds(0);
      ma.markers.push_back(m);
    }
    
    if(!latest_drone_poses_ || latest_drone_poses_->poses.empty()){
      marker_array_pub_->publish(ma); delete octree; return;
    }
    // Agrupa as poses em zonas (Zona1: r<d0, Zona2: [d0,2*d0), etc.)
    std::map<int, std::vector<Vector3>> zones;
    for(auto &p: latest_drone_poses_->poses){
      Vector3 pos(p.position.x, p.position.y, p.position.z);
      int z = (pos.norm() < d0)? 1: int(std::floor(pos.norm()/d0)) + 1;
      zones[z].push_back(pos);
    }
    int max_zone = 0; for(auto &z: zones) max_zone = std::max(max_zone, z.first);
    
    // Calcula os hotspots (baseados nas poses da zona i+1, projetadas na esfera de raio i*d0)
    std::vector<Hotspot> hotspots;
    for(int i=1;i<max_zone;i++){
      int zone = i+1; if(zones.find(zone)==zones.end()) continue;
      std::vector<Vector3> inter;
      for(auto &p: zones[zone]){
        Vector3 ip = p.normalized()*(i*d0);
        Vector3 dir = ip - p; double rl = dir.norm();
        Vector3 rdir = rl<1e-6? Vector3(): dir.normalized();
        octomap::point3d end;
        if(!octree->castRay(octomap::point3d(p.x,p.y,p.z),
              octomap::point3d(rdir.x, rdir.y, rdir.z), end, true, rl))
          inter.push_back(ip);
      }
      auto clusters = clusterPoints(inter, 2.0);
      for(auto &cl: clusters) {
        if(cl.empty()) continue;
        Vector3 sum; for(auto &pt: cl) sum = sum+pt;
        Hotspot hs; hs.center = sum*(1.0/cl.size()); hs.cluster_size = cl.size(); hs.sphere_index = i;
        hotspots.push_back(hs);
      }
    }
    
    // Mapa de conexão por zona (true se a pose está conectada)
    std::map<int, std::vector<bool>> conn;
    for(auto &z: zones) { conn[z.first] = std::vector<bool>(z.second.size(), false); }
    
    // Lambda para verificar linha de visão (LOS)
    auto freeLOS = [&](const Vector3 &a, const Vector3 &b)->bool {
      Vector3 d = b-a; double rl = d.norm(); Vector3 rdir = rl<1e-6? Vector3(): d.normalized();
      octomap::point3d end;
      return !octree->castRay(octomap::point3d(a.x,a.y,a.z),
                               octomap::point3d(rdir.x,rdir.y,rdir.z),
                               end, true, rl);
    };
    
    // Cria relações (primeira conexão) entre hotspots e poses nas zonas (para z>=2)
    visualization_msgs::msg::Marker rel;
    rel.header.frame_id = msg->header.frame_id; rel.header.stamp = this->now();
    rel.ns="relations"; rel.id = mid++; rel.type = visualization_msgs::msg::Marker::LINE_LIST;
    rel.action = visualization_msgs::msg::Marker::ADD; rel.scale.x = 0.1;
    rel.color.r=0.0; rel.color.g=1.0; rel.color.b=0.0; rel.color.a=1.0;
    for(auto &zp: zones){
      int z = zp.first; if(z<=1) continue;
      int sph = z-1;
      for(size_t i=0;i<zp.second.size();i++){
        Vector3 pos = zp.second[i];
        Hotspot* best = nullptr;
        for(auto &hs: hotspots){
          if(hs.sphere_index==sph && (pos-hs.center).norm()<d0 && freeLOS(hs.center, pos)){
            if(!best || hs.cluster_size > best->cluster_size) best = &hs;
          }
        }
        if(best){ best->connection_count++; conn[z][i]=true;
          geometry_msgs::msg::Point p1, p2; 
          p1.x=best->center.x; p1.y=best->center.y; p1.z=best->center.z;
          p2.x=pos.x; p2.y=pos.y; p2.z=pos.z;
          rel.points.push_back(p1); rel.points.push_back(p2);
        }
      }
    }
    if(!rel.points.empty()){ rel.lifetime = rclcpp::Duration::from_seconds(0); ma.markers.push_back(rel); }
    
    // Gera subhotspots para TODAS as zonas (z>=2) até que todas as poses estejam conectadas
    for(auto &zp: zones) {
      int z = zp.first; if(z<2) continue;
      bool progress = true;
      while(progress) {
        progress = false;
        int bestIdx = -1, bestCount = 0;
        // Seleciona candidato dentre das poses já conectadas
        for(size_t j=0;j<zp.second.size(); j++){
          if(conn[z][j]){
            int cnt = 0;
            for(size_t i=0;i<zp.second.size(); i++){
              if(!conn[z][i] && (zp.second[i]-zp.second[j]).norm()<d0 && freeLOS(zp.second[j], zp.second[i]))
                cnt++;
            }
            if(cnt>bestCount){ bestCount = cnt; bestIdx = j; }
          }
        }
        if(bestIdx!=-1 && bestCount>0){
          Vector3 subhot = zp.second[bestIdx];
          visualization_msgs::msg::Marker subM;
          subM.header.frame_id = msg->header.frame_id; subM.header.stamp = this->now();
          subM.ns="subhotspots"; subM.id = mid++; subM.type = visualization_msgs::msg::Marker::SPHERE;
          subM.action = visualization_msgs::msg::Marker::ADD; subM.pose.orientation.w = 1.0;
          subM.pose.position.x = subhot.x; subM.pose.position.y = subhot.y; subM.pose.position.z = subhot.z;
          subM.scale.x = 1.0; subM.scale.y = 1.0; subM.scale.z = 1.0;
          subM.color.r = 1.0; subM.color.g = 0.5; subM.color.b = 0.0; subM.color.a = 1.0;
          subM.lifetime = rclcpp::Duration::from_seconds(0);
          ma.markers.push_back(subM);
          visualization_msgs::msg::Marker subRel;
          subRel.header.frame_id = msg->header.frame_id; subRel.header.stamp = this->now();
          subRel.ns = "sub_relations"; subRel.id = mid++; subRel.type = visualization_msgs::msg::Marker::LINE_LIST;
          subRel.action = visualization_msgs::msg::Marker::ADD; subRel.scale.x = 0.1;
          subRel.color.r = 1.0; subRel.color.g = 0.0; subRel.color.b = 1.0; subRel.color.a = 1.0;
          for(size_t i=0;i<zp.second.size(); i++){
            if(!conn[z][i] && (zp.second[i]-subhot).norm()<d0 && freeLOS(subhot, zp.second[i])){
              geometry_msgs::msg::Point p1, p2; 
              p1.x = subhot.x; p1.y = subhot.y; p1.z = subhot.z;
              p2.x = zp.second[i].x; p2.y = zp.second[i].y; p2.z = zp.second[i].z;
              subRel.points.push_back(p1); subRel.points.push_back(p2);
              conn[z][i] = true; progress = true;
            }
          }
          if(!subRel.points.empty()){
            subRel.lifetime = rclcpp::Duration::from_seconds(0);
            ma.markers.push_back(subRel);
          }
        }
      }
    }
    
    // Publica apenas os hotspots que tiveram alguma conexão
    for(auto &hs: hotspots){
      if(hs.connection_count > 0){
        visualization_msgs::msg::Marker hm;
        hm.header.frame_id = msg->header.frame_id; hm.header.stamp = this->now();
        hm.ns = "hotspots"; hm.id = mid++; hm.type = visualization_msgs::msg::Marker::SPHERE;
        hm.action = visualization_msgs::msg::Marker::ADD; hm.pose.orientation.w = 1.0;
        hm.pose.position.x = hs.center.x; hm.pose.position.y = hs.center.y; hm.pose.position.z = hs.center.z;
        hm.scale.x = 1.0; hm.scale.y = 1.0; hm.scale.z = 1.0;
        hm.color.r = 0.0; hm.color.g = 0.0; hm.color.b = 1.0; hm.color.a = 1.0;
        hm.lifetime = rclcpp::Duration::from_seconds(0);
        ma.markers.push_back(hm);
      }
    }
    
    marker_array_pub_->publish(ma);
    delete octree;
  }
  
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr drone_poses_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  geometry_msgs::msg::PoseArray::SharedPtr latest_drone_poses_;
};

int main(int argc, char** argv){
  rclcpp::init(argc,argv);
  auto node = std::make_shared<WifiRangeVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

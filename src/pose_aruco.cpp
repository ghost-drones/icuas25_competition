#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "icuas25_msgs/msg/target_info.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <unordered_map>
#include <vector>
#include <string>
#include <cstdlib>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ArucoProcessor : public rclcpp::Node
{
public:
    ArucoProcessor()
        : Node("aruco_processor"),
          tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        // Obtém o número de robôs da variável de ambiente
        const char *num_robots_env = std::getenv("NUM_ROBOTS");
        num_robots_ = num_robots_env ? std::stoi(num_robots_env) : 5;

        // Inicializa publicadores
        publisher_ = this->create_publisher<icuas25_msgs::msg::TargetInfo>("/target_found", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ghost/visualization_aruco_markers", 10);

        // Subscreve para cada drone
        for (int i = 1; i <= num_robots_; ++i)
        {
            std::string robot_prefix = "cf_" + std::to_string(i);
            active_markers_[i] = {};

            // Assinatura para marcadores ArUco
            auto aruco_callback = [this, i](ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
            { markers_callback(msg, i); };

            this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
                "/" + robot_prefix + "/aruco_markers", 10, aruco_callback);

            // Assinatura para a pose do drone
            auto pose_callback = [this, i](geometry_msgs::msg::PoseStamped::SharedPtr msg)
            { drone_pose_callback(msg, i); };

            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/" + robot_prefix + "/pose", 100, pose_callback);
        }

        // Timer para verificar marcadores expirados a cada 0.1s
        timer_ = this->create_wall_timer(100ms, std::bind(&ArucoProcessor::check_expired_markers, this));
    }

private:
    int num_robots_;
    std::unordered_map<int, geometry_msgs::msg::Pose> drone_pose_;
    std::unordered_map<int, std::unordered_map<int, std::pair<double, geometry_msgs::msg::Pose>>> active_markers_;

    rclcpp::Publisher<icuas25_msgs::msg::TargetInfo>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    void drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int robot_id)
    {
        // Atualiza a pose do drone e publica a TF da câmera
        drone_pose_[robot_id] = msg->pose;
        publish_camera_tf(robot_id);
    }

    void markers_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg, int robot_id)
    {
        double current_time = this->get_clock()->now().seconds();

        for (size_t i = 0; i < msg->marker_ids.size(); ++i)
        {
            int marker_id = msg->marker_ids[i];
            active_markers_[robot_id][marker_id] = {current_time, msg->poses[i]};
            publish_tf(msg->poses[i], marker_id, robot_id);
        }
    }

    void check_expired_markers()
    {
        double current_time = this->get_clock()->now().seconds();

        for (auto &[robot_id, markers] : active_markers_)
        {
            std::vector<int> expired_ids;

            for (auto &[marker_id, data] : markers)
            {
                double last_time = data.first;
                const auto &pose = data.second;

                if (current_time - last_time > 1.0)
                {
                    publish_target_info(pose.position.x, pose.position.y, pose.position.z, marker_id, robot_id);
                    expired_ids.push_back(marker_id);
                }
            }

            for (int id : expired_ids)
            {
                markers.erase(id);
            }
        }
    }

    void publish_target_info(double x, double y, double z, int marker_id, int robot_id)
    {
        geometry_msgs::msg::TransformStamped transform;

        try
        {
            transform = tf_buffer_->lookupTransform(
                "world",
                "cf_" + std::to_string(robot_id) + "/aruco_" + std::to_string(marker_id),
                rclcpp::Time(0));
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Falha ao buscar transformação para cf_%d/aruco_%d: %s", robot_id, marker_id, e.what());
            return;
        }

        auto target_info = icuas25_msgs::msg::TargetInfo();
        target_info.id = marker_id;
        target_info.location.x = transform.transform.translation.x;
        target_info.location.y = transform.transform.translation.y;
        target_info.location.z = transform.transform.translation.z;
        publisher_->publish(target_info);

        // Publica marcador para RViz
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = "world";
        marker.ns = "aruco_markers";
        marker.id = robot_id * 1000 + marker_id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = transform.transform.translation;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        marker.lifetime.sec = 3;
        marker_pub_->publish(marker);
    }

    void publish_tf(const geometry_msgs::msg::Pose &pose, int aruco_id, int robot_id)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "cf_" + std::to_string(robot_id) + "/camera_frame";
        t.child_frame_id = "cf_" + std::to_string(robot_id) + "/aruco_" + std::to_string(aruco_id);

        t.transform.translation = pose.position;
        t.transform.rotation = pose.orientation;
        tf_broadcaster_->sendTransform(t);
    }

    void publish_camera_tf(int robot_id)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "cf_" + std::to_string(robot_id);
        t.child_frame_id = "cf_" + std::to_string(robot_id) + "/camera_frame";

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = -0.01;

        tf2::Quaternion q;
        q.setRPY(1.57, 0, 1.57);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

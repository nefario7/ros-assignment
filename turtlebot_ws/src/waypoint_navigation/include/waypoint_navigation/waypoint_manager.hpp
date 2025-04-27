#ifndef WAYPOINT_NAVIGATION__WAYPOINT_MANAGER_HPP_
#define WAYPOINT_NAVIGATION__WAYPOINT_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <fstream>
#include <cmath>
#include <functional>

#include "waypoint_navigation/waypoint_utils.hpp"
#include "waypoint_navigation/types/waypoint_type.hpp"
#include "waypoint_navigation/srv/create_waypoint.hpp"

namespace waypoint_navigation
{

    class WaypointManager : public rclcpp::Node
    {
    public:
        explicit WaypointManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        void loadWaypoints();
        void saveWaypoints();
        void addWaypoint(const std::string &id, const MsgPoseStamped &pose, WaypointType type);
        bool isNearExistingWaypoint(const MsgPose &current_pose);
        void publishWaypointMarkers();

        // Callbacks
        void handleCreateWaypointService(
            const std::shared_ptr<waypoint_navigation::srv::CreateWaypoint::Request> request,
            std::shared_ptr<waypoint_navigation::srv::CreateWaypoint::Response> response);

        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    private:
        std::unordered_map<std::string, Waypoint> waypoints_;
        std::string waypoint_file_path_;
        std::string map_frame_;
        std::string robot_base_frame_;

        // Automatic creation parameters
        double auto_creation_distance_threshold_;
        double auto_creation_angle_threshold_rad_;
        double proximity_threshold_;

        // Automatic creation state
        MsgPose last_pose_for_auto_creation_;
        bool initial_pose_received_ = false;

        // TF2
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // ROS Interfaces
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Service<waypoint_navigation::srv::CreateWaypoint>::SharedPtr create_waypoint_service_;
    };

} // namespace waypoint_navigation

#endif // WAYPOINT_NAVIGATION__WAYPOINT_MANAGER_HPP_
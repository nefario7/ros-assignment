#include "waypoint_navigation/waypoint_manager.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "waypoint_navigation/waypoint_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
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

namespace waypoint_navigation
{

    WaypointManager::WaypointManager(const rclcpp::NodeOptions &options)
        : Node("waypoint_manager", options)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing WaypointManager Node...");

        // --- Declare and Load Parameters ---
        RCLCPP_INFO(this->get_logger(), "Declaring and loading parameters...");

        // Auto-creation parameters
        rcl_interfaces::msg::ParameterDescriptor distance_desc;
        distance_desc.description = "Distance threshold in meters for automatic waypoint creation.";
        this->declare_parameter("auto_creation_distance_threshold", 2.0, distance_desc);
        auto_creation_distance_threshold_ = this->get_parameter("auto_creation_distance_threshold").get_value<double>();

        rcl_interfaces::msg::ParameterDescriptor angle_desc;
        angle_desc.description = "Angle threshold in degrees for automatic waypoint creation.";
        this->declare_parameter("auto_creation_angle_threshold_deg", 45.0, angle_desc);
        double angle_deg = this->get_parameter("auto_creation_angle_threshold_deg").get_value<double>();
        auto_creation_angle_threshold_rad_ = angle_deg * M_PI / 180.0;

        rcl_interfaces::msg::ParameterDescriptor proximity_desc;
        proximity_desc.description = "Proximity threshold in meters to check for existing waypoints.";
        this->declare_parameter("proximity_threshold", 0.5, proximity_desc);
        proximity_threshold_ = this->get_parameter("proximity_threshold").get_value<double>();

        // File path and frame parameters
        rcl_interfaces::msg::ParameterDescriptor file_path_desc;
        file_path_desc.description = "Full path to the file for saving/loading waypoints.";
        this->declare_parameter("waypoint_file_path", std::string("waypoints.yaml"), file_path_desc);
        waypoint_file_path_ = this->get_parameter("waypoint_file_path").get_value<std::string>();

        rcl_interfaces::msg::ParameterDescriptor map_frame_desc;
        map_frame_desc.description = "The fixed frame used for waypoint coordinates.";
        this->declare_parameter("map_frame", std::string("map"), map_frame_desc);
        map_frame_ = this->get_parameter("map_frame").get_value<std::string>();

        rcl_interfaces::msg::ParameterDescriptor robot_frame_desc;
        robot_frame_desc.description = "The base frame of the robot.";
        this->declare_parameter("robot_base_frame", std::string("base_link"), robot_frame_desc);
        robot_base_frame_ = this->get_parameter("robot_base_frame").get_value<std::string>();

        RCLCPP_INFO(this->get_logger(), "Parameters Loaded:");
        RCLCPP_INFO(this->get_logger(), "  waypoint_file_path: %s", waypoint_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  map_frame: %s", map_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  robot_base_frame: %s", robot_base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  proximity_threshold: %.2f", proximity_threshold_);
        RCLCPP_INFO(this->get_logger(), "  auto_creation_distance_threshold: %.2f", auto_creation_distance_threshold_);
        RCLCPP_INFO(this->get_logger(), "  auto_creation_angle_threshold_deg: %.1f", angle_deg);

        // --- TF2 Setup ---
        RCLCPP_INFO(this->get_logger(), "Initializing TF2 buffer and listener...");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // --- Load Initial Waypoints ---
        loadWaypoints(); // Uses loaded file path and map frame parameters

        // --- Initialize ROS Interfaces ---
        RCLCPP_INFO(this->get_logger(), "Setting up ROS services, subscribers, and publishers...");

        // Service for manual waypoint creation
        create_waypoint_service_ = this->create_service<waypoint_navigation::srv::CreateWaypoint>(
            "~/create_waypoint",
            std::bind(&WaypointManager::handleCreateWaypointService, this, std::placeholders::_1, std::placeholders::_2));

        // Subscriber for automatic waypoint creation based on odometry
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            rclcpp::SystemDefaultsQoS(), // Consider adjusting QoS based on odom source reliability
            std::bind(&WaypointManager::odomCallback, this, std::placeholders::_1));

        // Publisher for waypoint visualization markers
        waypoint_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/waypoints_viz",
            rclcpp::SystemDefaultsQoS()); // Consider Transient Local QoS if markers needed on late joiners

        // Publish markers for initially loaded waypoints
        publishWaypointMarkers();

        RCLCPP_INFO(this->get_logger(), "WaypointManager Node initialized successfully.");
    }

    void WaypointManager::loadWaypoints()
    {
        waypoints_ = loadWaypointsFromYaml(waypoint_file_path_, map_frame_, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Loaded %zu waypoints.", waypoints_.size());
    }

    void WaypointManager::saveWaypoints()
    {
        RCLCPP_INFO(this->get_logger(), "Saving %zu waypoints to %s...", waypoints_.size(), waypoint_file_path_.c_str());
        YAML::Node data;
        YAML::Node wp_list;

        for (const auto &pair : waypoints_)
        {
            const auto &wp = pair.second;
            YAML::Node node;
            node["id"] = wp.id;
            node["frame_id"] = wp.pose.header.frame_id;
            // Position
            node["pose"]["position"]["x"] = wp.pose.pose.position.x;
            node["pose"]["position"]["y"] = wp.pose.pose.position.y;
            node["pose"]["position"]["z"] = wp.pose.pose.position.z;
            // Orientation
            node["pose"]["orientation"]["x"] = wp.pose.pose.orientation.x;
            node["pose"]["orientation"]["y"] = wp.pose.pose.orientation.y;
            node["pose"]["orientation"]["z"] = wp.pose.pose.orientation.z;
            node["pose"]["orientation"]["w"] = wp.pose.pose.orientation.w;
            // Type
            node["type"] = (wp.type == WaypointType::MANUAL) ? "MANUAL" : "AUTOMATIC";
            wp_list.push_back(node);
        }

        data["waypoints"] = wp_list;

        try
        {
            std::ofstream fout(waypoint_file_path_);
            fout << data;
            RCLCPP_INFO(this->get_logger(), "Waypoints saved successfully.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to waypoint file [%s]. Error: %s", waypoint_file_path_.c_str(), e.what());
        }
    }

    bool WaypointManager::isNearExistingWaypoint(const MsgPose &current_pose)
    {
        for (const auto &pair : waypoints_)
        {
            const auto &wp_pose = pair.second.pose.pose;
            double dist = calculateDistance(current_pose.position, wp_pose.position);
            if (dist < proximity_threshold_)
            {
                RCLCPP_DEBUG(this->get_logger(), "Waypointt Near existing waypoint %s (distance: %.2fm)", pair.first.c_str(), dist);
                return true;
            }
        }
        return false;
    }

    void WaypointManager::addWaypoint(const std::string &id, const MsgPoseStamped &pose, WaypointType type)
    {
        if (waypoints_.count(id))
        {
            RCLCPP_WARN(this->get_logger(), "Waypoint ID '%s' already exists. Not adding.", id.c_str());
            return;
        }

        // Check if the pose is in the map frame, if not, transform it
        MsgPoseStamped pose_in_map_frame;
        if (pose.header.frame_id != map_frame_)
            tf_buffer_->transform(pose, pose_in_map_frame, map_frame_, tf2::durationFromSec(0.5));
        else
            pose_in_map_frame = pose;

        Waypoint new_waypoint;
        new_waypoint.id = id;
        new_waypoint.pose = pose_in_map_frame;
        new_waypoint.type = type;

        waypoints_[id] = new_waypoint;
        RCLCPP_INFO(this->get_logger(), "Added %s waypoint '%s' at (%.2f, %.2f) in frame '%s'",
                    (type == WaypointType::MANUAL ? "Manual" : "Automatic"),
                    id.c_str(),
                    pose_in_map_frame.pose.position.x,
                    pose_in_map_frame.pose.position.y,
                    pose_in_map_frame.header.frame_id.c_str());

        publishWaypointMarkers();
        saveWaypoints();
    }

    namespace
    {
        visualization_msgs::msg::Marker createWaypointMarker(
            const Waypoint &wp,
            int marker_id,
            const rclcpp::Time &now)
        {
            visualization_msgs::msg::Marker marker;
            marker.header = wp.pose.header;
            marker.header.stamp = now;
            marker.ns = "waypoints";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = wp.pose.pose;
            marker.scale.x = 0.5;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;

            if (wp.type == WaypointType::MANUAL)
            {
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
            }
            else
            {
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
            }
            marker.lifetime = rclcpp::Duration(0, 0);
            return marker;
        }

        visualization_msgs::msg::Marker createWaypointTextMarker(
            const Waypoint &wp,
            int marker_id)
        {
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = wp.pose.header;
            text_marker.ns = "waypoint_labels";
            text_marker.id = marker_id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose = wp.pose.pose;
            text_marker.pose.position.z += 0.3;
            text_marker.scale.z = 0.2;
            text_marker.color.a = 1.0;
            text_marker.color.r = 0.0f;
            text_marker.color.g = 0.0f;
            text_marker.color.b = 0.0f;
            text_marker.text = wp.id;
            text_marker.lifetime = rclcpp::Duration(0, 0);
            return text_marker;
        }
    }

    void WaypointManager::publishWaypointMarkers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int marker_id = 0;

        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = map_frame_;
        delete_marker.header.stamp = this->get_clock()->now();
        delete_marker.ns = "waypoints";
        delete_marker.id = -1;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        for (const auto &pair : waypoints_)
        {
            // Create marker for waypoint
            auto marker = createWaypointMarker(pair.second, marker_id++, this->get_clock()->now());
            marker_array.markers.push_back(marker);

            // Create text marker for waypoint label
            auto text_marker = createWaypointTextMarker(pair.second, marker_id++);
            marker_array.markers.push_back(text_marker);
        }

        waypoint_marker_publisher_->publish(marker_array);
        RCLCPP_DEBUG(this->get_logger(), "Published %zu markers.", marker_array.markers.size());
    }

    // --- Callbacks Implementation ---
    void WaypointManager::handleCreateWaypointService(
        const std::shared_ptr<waypoint_navigation::srv::CreateWaypoint::Request> request,
        std::shared_ptr<waypoint_navigation::srv::CreateWaypoint::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "CreateWaypoint service called with id: %s", request->waypoint_id.c_str());

        if (request->waypoint_id.empty())
        {
            response->success = false;
            response->message = "Waypoint ID cannot be empty.";
            RCLCPP_WARN(this->get_logger(), response->message.c_str());
            return;
        }

        if (waypoints_.count(request->waypoint_id))
        {
            response->success = false;
            response->message = "Waypoint ID '" + request->waypoint_id + "' already exists.";
            RCLCPP_WARN(this->get_logger(), response->message.c_str());
            return;
        }

        MsgPoseStamped current_pose_stamped;
        MsgTransformStamped transform_stamped;
        rclcpp::Time now = this->get_clock()->now();

        // Get the transform from the map frame to the robot base frame
        try
        {
            transform_stamped = tf_buffer_->lookupTransform(
                map_frame_, robot_base_frame_, tf2::TimePointZero);

            current_pose_stamped.header.stamp = now;
            current_pose_stamped.header.frame_id = map_frame_;
            current_pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
            current_pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
            current_pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
            current_pose_stamped.pose.orientation = transform_stamped.transform.rotation;

            RCLCPP_INFO(this->get_logger(), "Robot pose in '%s' frame: (%.2f, %.2f, %.2f)",
                        map_frame_.c_str(),
                        current_pose_stamped.pose.position.x,
                        current_pose_stamped.pose.position.y,
                        current_pose_stamped.pose.position.z);

            // Skip this check if the user is manually creating a waypoint
            // if (isNearExistingWaypoint(current_pose_stamped.pose))
            // {
            //     response->success = false;
            //     response->message = "Robot is too close to an existing waypoint. Manual creation skipped.";
            //     RCLCPP_WARN(this->get_logger(), response->message.c_str());
            //     return;
            // }

            addWaypoint(request->waypoint_id, current_pose_stamped, WaypointType::MANUAL);
            response->success = true;
            response->message = "Waypoint '" + request->waypoint_id + "' created manually.";
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform from '%s' to '%s': %s",
                         robot_base_frame_.c_str(), map_frame_.c_str(), ex.what());
            response->success = false;
            response->message = "Failed to get current robot pose: " + std::string(ex.what());
            return;
        }
    }

    void WaypointManager::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Assuming odom frame == map frame or TF is handled elsewhere for simplicity
        if (msg->header.frame_id != map_frame_)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Odom frame '%s' != map frame '%s'. Automatic waypoints might be incorrect if TF is not broadcasting odom->map correctly.",
                             msg->header.frame_id.c_str(), map_frame_.c_str());
        }

        if (!initial_pose_received_)
        {
            last_pose_for_auto_creation_ = msg->pose.pose;
            initial_pose_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial pose received for automatic waypoint creation.");
            return;
        }

        const auto &current_pose = msg->pose.pose;
        double distance_moved = calculateDistance(current_pose.position, last_pose_for_auto_creation_.position);
        double current_yaw = getYawFromQuaternion(current_pose.orientation);
        double last_yaw = getYawFromQuaternion(last_pose_for_auto_creation_.orientation);
        double angle_turned = std::abs(calculateYawDifference(current_yaw, last_yaw));

        bool create_waypoint = false;
        std::string reason = "";
        if (distance_moved >= auto_creation_distance_threshold_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Distance threshold (%.2fm) reached.", auto_creation_distance_threshold_);
            create_waypoint = true;
            reason = "distance";
        }
        else if (angle_turned >= auto_creation_angle_threshold_rad_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Angle threshold (%.2f rad / %.1f deg) reached.",
                         auto_creation_angle_threshold_rad_, auto_creation_angle_threshold_rad_ * 180.0 / M_PI);
            create_waypoint = true;
            reason = "angle";
        }

        if (create_waypoint)
        {
            if (!isNearExistingWaypoint(current_pose))
            {
                const std::string auto_id = "auto_wp_" + std::to_string(waypoints_.size() + 1);
                MsgPoseStamped current_pose_stamped;
                current_pose_stamped.header = msg->header;
                // Important: Ensure the frame_id is map_frame_ if odom != map_frame
                if (msg->header.frame_id != map_frame_)
                {
                    current_pose_stamped.header.frame_id = msg->header.frame_id;
                }
                else
                {
                    current_pose_stamped.header.frame_id = map_frame_;
                }
                current_pose_stamped.pose = current_pose;

                addWaypoint(auto_id, current_pose_stamped, WaypointType::AUTOMATIC);
                last_pose_for_auto_creation_ = current_pose;
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Near existing waypoint, skipping automatic creation triggered by %s.", reason.c_str());
                last_pose_for_auto_creation_ = current_pose;
            }
        }
    }

} // namespace waypoint_navigation

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_navigation::WaypointManager)

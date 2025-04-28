#ifndef WAYPOINT_NAVIGATION__WAYPOINT_NAVIGATOR_HPP_
#define WAYPOINT_NAVIGATION__WAYPOINT_NAVIGATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "waypoint_navigation/waypoint_manager.hpp"                  // For Waypoint struct
#include "waypoint_navigation/waypoint_utils.hpp"                    // For utilities
#include "waypoint_navigation/action/navigate_through_waypoints.hpp" // Include the action definition

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <optional>

namespace waypoint_navigation
{

    class WaypointNavigator : public rclcpp::Node
    {
    public:
        // Define Action types
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
        using NavigateThroughWaypoints = waypoint_navigation::action::NavigateThroughWaypoints;
        using GoalHandleNavigateThroughWaypoints = rclcpp_action::ServerGoalHandle<NavigateThroughWaypoints>;

        explicit WaypointNavigator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        // Action Server Callbacks
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const NavigateThroughWaypoints::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleNavigateThroughWaypoints> goal_handle);
        void handle_accepted(
            const std::shared_ptr<GoalHandleNavigateThroughWaypoints> goal_handle);
        void execute_goal();

        // Action Client Callbacks (for Nav2 interaction)
        void nav2_goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle);
        void nav2_feedback_callback(
            GoalHandleNavigateToPose::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void nav2_result_callback(const GoalHandleNavigateToPose::WrappedResult &result);

        // Core Logic
        void loadWaypoints();

        // Member Variables
        std::unordered_map<std::string, Waypoint> waypoints_;
        std::string waypoint_file_path_;
        std::string map_frame_;
        std::string robot_base_frame_;

        // Action Client for Nav2
        rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_action_client_;
        rclcpp_action::Server<NavigateThroughWaypoints>::SharedPtr navigate_action_server_;

        // TF2
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // State for tracking the current goal handle
        std::shared_ptr<GoalHandleNavigateThroughWaypoints> current_goal_handle_ = nullptr;
        std::shared_ptr<GoalHandleNavigateToPose> nav2_goal_handle_ = nullptr;
        rclcpp::TimerBase::SharedPtr execution_timer_ = nullptr;

        std::vector<std::string> current_navigation_plan_;
        size_t current_plan_index_ = 0;
        double waypoint_connection_threshold_ = 5.0;

        MsgPoseStamped getCurrentRobotPose();
        std::optional<std::string> findNearestWaypoint(const MsgPoseStamped &current_pose);
        void navigateToNextWaypointInPlan();
    };

} // namespace waypoint_navigation

#endif // WAYPOINT_NAVIGATION__WAYPOINT_NAVIGATOR_HPP_
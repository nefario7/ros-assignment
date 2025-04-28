#include "waypoint_navigation/waypoint_navigator.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <limits>
#include <optional>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <string>

#include "waypoint_navigation/path_planner.hpp"

using namespace std::placeholders;

namespace waypoint_navigation
{

    WaypointNavigator::WaypointNavigator(const rclcpp::NodeOptions &options)
        : Node("waypoint_navigator", options)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing WaypointNavigator Node...");

        // Declare and Load Parameters
        RCLCPP_INFO(this->get_logger(), "Declaring and loading parameters...");

        // Waypoint File Path
        rcl_interfaces::msg::ParameterDescriptor file_path_desc;
        file_path_desc.description = "Path to the waypoints file.";
        this->declare_parameter("waypoint_file_path", std::string("waypoints.yaml"), file_path_desc);
        waypoint_file_path_ = this->get_parameter("waypoint_file_path").get_value<std::string>();

        // Map Frame
        rcl_interfaces::msg::ParameterDescriptor map_frame_desc;
        map_frame_desc.description = "The fixed frame used for waypoint coordinates.";
        this->declare_parameter("map_frame", std::string("map"), map_frame_desc);
        map_frame_ = this->get_parameter("map_frame").get_value<std::string>();

        // Robot Base Frame
        rcl_interfaces::msg::ParameterDescriptor robot_frame_desc;
        robot_frame_desc.description = "The base frame of the robot.";
        this->declare_parameter("robot_base_frame", std::string("base_link"), robot_frame_desc);
        robot_base_frame_ = this->get_parameter("robot_base_frame").get_value<std::string>();

        // Waypoint Connection Threshold
        rcl_interfaces::msg::ParameterDescriptor connection_thresh_desc;
        connection_thresh_desc.description = "Maximum distance between waypoints to consider them directly connected for path planning.";
        this->declare_parameter("waypoint_connection_threshold", 5.0, connection_thresh_desc);
        waypoint_connection_threshold_ = this->get_parameter("waypoint_connection_threshold").get_value<double>();

        RCLCPP_INFO(this->get_logger(), "Parameters loaded.");
        RCLCPP_INFO(this->get_logger(), "  waypoint_file_path: %s", waypoint_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  map_frame: %s", map_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  robot_base_frame: %s", robot_base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  waypoint_connection_threshold: %.2f", waypoint_connection_threshold_);

        // Load Initial Waypoints
        loadWaypoints();

        // Initialize TF2
        RCLCPP_INFO(this->get_logger(), "Initializing TF2 buffer and listener...");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize Action Client (Nav2)
        RCLCPP_INFO(this->get_logger(), "Creating Action Client for 'navigate_to_pose'...");
        nav2_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server 'navigate_to_pose'...");
        if (!nav2_action_client_->wait_for_action_server(std::chrono::seconds(20)))
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server 'navigate_to_pose' not available after waiting 20s. Shutting down node.");
            if (rclcpp::ok())
                rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Connected to Nav2 action server 'navigate_to_pose'.");

        // Initialize Action Server (Waypoint Navigation)
        RCLCPP_INFO(this->get_logger(), "Creating Action Server for '~/navigate_through_waypoints'...");
        navigate_action_server_ = rclcpp_action::create_server<NavigateThroughWaypoints>(
            this,
            "~/navigate_through_waypoints",
            std::bind(&WaypointNavigator::handle_goal, this, _1, _2),
            std::bind(&WaypointNavigator::handle_cancel, this, _1),
            std::bind(&WaypointNavigator::handle_accepted, this, _1));
        RCLCPP_INFO(this->get_logger(), "'~/navigate_through_waypoints' action server started.");

        RCLCPP_INFO(this->get_logger(), "WaypointNavigator Node initialized successfully.");
    }

    void WaypointNavigator::loadWaypoints()
    {
        waypoints_ = loadWaypointsFromYaml(waypoint_file_path_, map_frame_, this->get_logger());
        RCLCPP_DEBUG(this->get_logger(), "Loaded %zu waypoints.", waypoints_.size());
        if (waypoints_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Waypoint file was empty or not found. Navigator cannot navigate until waypoints are added.");
        }
    }

    // Action Server Callbacks
    rclcpp_action::GoalResponse WaypointNavigator::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const NavigateThroughWaypoints::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received navigation goal request for waypoint '%s'", goal->target_waypoint_id.c_str());
        (void)uuid;

        RCLCPP_INFO(this->get_logger(), "Reloading waypoints before goal check...");
        loadWaypoints();

        // Check existence
        if (waypoints_.find(goal->target_waypoint_id) == waypoints_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Requested target wayapoint '%s' does not exist.", goal->target_waypoint_id.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Check if active
        if (current_goal_handle_ && current_goal_handle_->is_active())
        {
            RCLCPP_WARN(this->get_logger(), "Another navigation goal is already active. Rejecting new goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Accepting navigation goal for waypoint '%s'", goal->target_waypoint_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse WaypointNavigator::handle_cancel(
        const std::shared_ptr<GoalHandleNavigateThroughWaypoints> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel navigation goal.");
        (void)goal_handle;

        if (nav2_goal_handle_)
        {
            RCLCPP_INFO(this->get_logger(), "Requesting cancellation of current Nav2 goal.");
            nav2_action_client_->async_cancel_goal(nav2_goal_handle_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Cancel request received, but no active Nav2 goal handle found.");
        }

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void WaypointNavigator::handle_accepted(
        const std::shared_ptr<GoalHandleNavigateThroughWaypoints> goal_handle)
    {
        current_goal_handle_ = goal_handle;
        RCLCPP_INFO(this->get_logger(), "Goal %s accepted. Scheduling execution.", rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());

        execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            std::bind(&WaypointNavigator::execute_goal, this));
    }

    void WaypointNavigator::execute_goal()
    {
        if (execution_timer_)
        {
            execution_timer_->cancel();
        }

        if (!current_goal_handle_ || !current_goal_handle_->is_active())
        {
            RCLCPP_WARN(this->get_logger(), "Execute goal called but current goal is no longer active.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Reloading waypoints before planning...");
        loadWaypoints();
        if (waypoints_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints or waypoint file is empty. Aborting goal.");
            auto result = std::make_shared<NavigateThroughWaypoints::Result>();
            result->success = false;
            result->message = "Failed to load waypoints before planning.";
            current_goal_handle_->abort(result);
            current_goal_handle_ = nullptr;
            return;
        }

        // Get the goal from the stored handle
        const auto goal = current_goal_handle_->get_goal();
        const std::string target_waypoint_id = goal->target_waypoint_id;
        RCLCPP_INFO(this->get_logger(), "Executing navigation goal to waypoint: %s", target_waypoint_id.c_str());

        // 1. Get current robot pose
        MsgPoseStamped current_pose = getCurrentRobotPose();
        if (current_pose.header.frame_id.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get current robot pose. Aborting goal.");
            auto result = std::make_shared<NavigateThroughWaypoints::Result>();
            result->success = false;
            result->message = "Failed to get current robot pose.";
            current_goal_handle_->abort(result);
            current_goal_handle_ = nullptr;
            return;
        }

        // 2. Find the nearest waypoint to start from
        auto start_waypoint_id_opt = findNearestWaypoint(current_pose);
        if (!start_waypoint_id_opt)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not find nearest waypoint to start from. Aborting goal.");
            auto result = std::make_shared<NavigateThroughWaypoints::Result>();
            result->success = false;
            result->message = "Failed to find nearest starting waypoint.";
            current_goal_handle_->abort(result);
            current_goal_handle_ = nullptr;
            return;
        }
        std::string start_waypoint_id = start_waypoint_id_opt.value();
        RCLCPP_INFO(this->get_logger(), "Nearest start waypoint: %s", start_waypoint_id.c_str());

        // 3. Plan the path through waypoints
        auto plan = planShortestPath(start_waypoint_id, target_waypoint_id, waypoints_, waypoint_connection_threshold_, this->get_logger());
        if (!plan.has_value())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not plan path from '%s' to '%s'. Aborting goal.",
                         start_waypoint_id.c_str(), target_waypoint_id.c_str());
            auto result = std::make_shared<NavigateThroughWaypoints::Result>();
            result->success = false;
            result->message = "Failed to plan waypoint path.";
            current_goal_handle_->abort(result);
            current_goal_handle_ = nullptr;
            return;
        }

        current_navigation_plan_ = plan.value();
        current_plan_index_ = 0;

        if (current_navigation_plan_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Planned path is empty. Aborting goal.");
            auto result = std::make_shared<NavigateThroughWaypoints::Result>();
            result->success = false;
            result->message = "Planned path was empty.";
            current_goal_handle_->abort(result);
            current_goal_handle_ = nullptr;
            return;
        }

        // 4. Start navigating the plan
        RCLCPP_INFO(this->get_logger(), "Starting navigation along the planned path of %zu waypoints.", current_navigation_plan_.size());
        navigateToNextWaypointInPlan();
    }

    // Action Client Callbacks (for Nav2 interaction)
    void WaypointNavigator::nav2_goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 goal was rejected by server");
            nav2_goal_handle_ = nullptr;
            if (current_goal_handle_ && current_goal_handle_->is_active())
            {
                auto result = std::make_shared<NavigateThroughWaypoints::Result>();
                result->success = false;
                result->message = "Nav2 rejected the goal.";
                current_goal_handle_->abort(result);
                current_goal_handle_ = nullptr;
                current_navigation_plan_.clear();
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Nav2 goal accepted by server, waiting for result");
            nav2_goal_handle_ = goal_handle;
        }
    }

    void WaypointNavigator::nav2_feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        // Publish feedback for the NavigateThroughWaypoints action if a goal is active
        if (current_goal_handle_ && current_goal_handle_->is_active())
        {
            auto feedback_msg = std::make_shared<NavigateThroughWaypoints::Feedback>();
            // Use the waypoint ID from the current step in the plan
            if (current_plan_index_ < current_navigation_plan_.size())
            {
                feedback_msg->current_waypoint_id = current_navigation_plan_[current_plan_index_];
            }
            else
            {
                // Should ideally not happen if result callback logic is correct, but provide fallback
                feedback_msg->current_waypoint_id = "<finalizing>";
            }
            feedback_msg->distance_remaining = feedback->distance_remaining;
            feedback_msg->estimated_time_remaining = rclcpp::Duration(feedback->estimated_time_remaining).seconds();
            current_goal_handle_->publish_feedback(feedback_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published feedback: Waypoint='%s', Dist=%.2f, Time=%.1f",
                         feedback_msg->current_waypoint_id.c_str(), feedback_msg->distance_remaining, feedback_msg->estimated_time_remaining);
        }
    }

    void WaypointNavigator::nav2_result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        // Ensure a goal from this node is active before processing result
        if (!current_goal_handle_ || !current_goal_handle_->is_active())
        {
            RCLCPP_WARN(this->get_logger(), "Received Nav2 result but no active NavigateThroughWaypoints goal handle stored. Ignoring.");
            return;
        }

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            std::string completed_waypoint_id = "unknown";
            if (current_plan_index_ < current_navigation_plan_.size())
            {
                completed_waypoint_id = current_navigation_plan_[current_plan_index_];
            }
            RCLCPP_INFO(this->get_logger(), "Successfully reached intermediate waypoint: %s (%zu/%zu)",
                        completed_waypoint_id.c_str(), current_plan_index_ + 1, current_navigation_plan_.size());
            current_plan_index_++;
            // Trigger navigation to the next waypoint in the plan
            navigateToNextWaypointInPlan();
            break; // Important: break here, overall success is handled by navigateToNextWaypointInPlan
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 goal was aborted while navigating the plan.");
            auto overall_result = std::make_shared<NavigateThroughWaypoints::Result>();
            overall_result->success = false;
            overall_result->message = "Navigation aborted by Nav2 during plan execution.";
            current_goal_handle_->abort(overall_result);
            current_goal_handle_ = nullptr;   // Clear handle on failure
            current_navigation_plan_.clear(); // Clear plan
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            RCLCPP_INFO(this->get_logger(), "Nav2 goal was canceled while navigating the plan.");
            auto overall_result = std::make_shared<NavigateThroughWaypoints::Result>();
            overall_result->success = false;
            overall_result->message = "Navigation plan canceled.";
            // Check if the cancellation was requested by our action client
            if (current_goal_handle_->is_canceling())
            {
                current_goal_handle_->canceled(overall_result);
            }
            else
            { // If Nav2 canceled for other reasons, treat as abort
                current_goal_handle_->abort(overall_result);
            }
            current_goal_handle_ = nullptr;   // Clear handle on failure/cancel
            current_navigation_plan_.clear(); // Clear plan
            break;
        }
        default:
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 goal failed with unknown result code during plan execution.");
            auto overall_result = std::make_shared<NavigateThroughWaypoints::Result>();
            overall_result->success = false;
            overall_result->message = "Navigation failed with unknown error during plan execution.";
            current_goal_handle_->abort(overall_result);
            current_goal_handle_ = nullptr;   // Clear handle on failure
            current_navigation_plan_.clear(); // Clear plan
            break;
        }
        }
        // Do NOT clear the current goal handle here on success,
        // it's cleared in navigateToNextWaypointInPlan when the *entire* plan succeeds.

        // Clear the Nav2 goal handle now that it has reached a terminal state
        nav2_goal_handle_ = nullptr;
    }

    MsgPoseStamped WaypointNavigator::getCurrentRobotPose()
    {
        MsgPoseStamped robot_pose;
        geometry_msgs::msg::TransformStamped transform_stamped;
        rclcpp::Time now = this->get_clock()->now();

        transform_stamped = tf_buffer_->lookupTransform(
            map_frame_, robot_base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5)); // Added timeout

        robot_pose.header.stamp = now; // Use current time for pose stamp
        robot_pose.header.frame_id = map_frame_;
        robot_pose.pose.position.x = transform_stamped.transform.translation.x;
        robot_pose.pose.position.y = transform_stamped.transform.translation.y;
        robot_pose.pose.position.z = transform_stamped.transform.translation.z;
        robot_pose.pose.orientation = transform_stamped.transform.rotation;

        RCLCPP_DEBUG(this->get_logger(), "Current robot pose in '%s': (%.2f, %.2f)",
                     map_frame_.c_str(), robot_pose.pose.position.x, robot_pose.pose.position.y);

        return robot_pose;
    }

    std::optional<std::string> WaypointNavigator::findNearestWaypoint(const MsgPoseStamped &current_pose)
    {
        if (waypoints_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Waypoint map is empty, cannot find nearest waypoint.");
            return std::nullopt;
        }

        if (current_pose.header.frame_id.empty() || current_pose.header.frame_id != map_frame_)
        {
            RCLCPP_ERROR(this->get_logger(), "Current robot pose is invalid or not in map frame ('%s'). Cannot find nearest waypoint.",
                         map_frame_.c_str());
            return std::nullopt;
        }

        std::string nearest_id = "";
        double min_dist_sq = std::numeric_limits<double>::max(); // Use squared distance to avoid sqrt

        for (const auto &pair : waypoints_)
        {
            const auto &wp = pair.second;
            // Ensure waypoint is also in the map frame (should be, but check)
            if (wp.pose.header.frame_id != map_frame_)
            {
                RCLCPP_WARN_ONCE(this->get_logger(), "Waypoint '%s' has frame '%s' but expected '%s'. Skipping for nearest check.",
                                 wp.id.c_str(), wp.pose.header.frame_id.c_str(), map_frame_.c_str());
                continue;
            }

            double dx = current_pose.pose.position.x - wp.pose.pose.position.x;
            double dy = current_pose.pose.position.y - wp.pose.pose.position.y;
            // Ignore Z for distance calculation on ground robot
            double dist_sq = dx * dx + dy * dy;

            if (dist_sq < min_dist_sq)
            {
                min_dist_sq = dist_sq;
                nearest_id = wp.id;
            }
        }

        if (nearest_id.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not find a nearest waypoint (this shouldn't happen if waypoints exist).");
            return std::nullopt;
        }

        RCLCPP_DEBUG(this->get_logger(), "Nearest waypoint to current pose is '%s' (distance: %.2fm).",
                     nearest_id.c_str(), std::sqrt(min_dist_sq));
        return nearest_id;
    }

    void WaypointNavigator::navigateToNextWaypointInPlan()
    {
        // Ensure the main goal is still active
        if (!current_goal_handle_ || !current_goal_handle_->is_active())
        {
            RCLCPP_WARN(this->get_logger(), "Main goal handle is not active. Stopping navigation sequence.");
            current_navigation_plan_.clear();
            return;
        }

        // Check if we have finished the plan
        if (current_plan_index_ >= current_navigation_plan_.size())
        {
            RCLCPP_INFO(this->get_logger(), "Successfully navigated the full waypoint plan.");
            auto result = std::make_shared<NavigateThroughWaypoints::Result>();
            result->success = true;
            result->message = "Navigation successful.";
            current_goal_handle_->succeed(result);
            current_goal_handle_ = nullptr;
            current_navigation_plan_.clear();
            return;
        }

        // Get the next waypoint ID and its pose
        const std::string next_waypoint_id = current_navigation_plan_[current_plan_index_];
        auto it = waypoints_.find(next_waypoint_id);
        if (it == waypoints_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Waypoint '%s' from plan not found in map. Aborting goal.", next_waypoint_id.c_str());
            auto result = std::make_shared<NavigateThroughWaypoints::Result>();
            result->success = false;
            result->message = "Waypoint from plan not found: " + next_waypoint_id;
            current_goal_handle_->abort(result);
            current_goal_handle_ = nullptr;
            current_navigation_plan_.clear();
            return;
        }
        const auto target_pose = it->second.pose;

        // Send the goal to Nav2
        auto nav2_goal_msg = NavigateToPose::Goal();
        nav2_goal_msg.pose = target_pose;
        // nav2_goal_msg.behavior_tree = ""; // Optional: Specify behavior tree

        RCLCPP_INFO(this->get_logger(), "Navigating to waypoint %zu/%zu: '%s' at (%.2f, %.2f)",
                    current_plan_index_ + 1,
                    current_navigation_plan_.size(),
                    next_waypoint_id.c_str(),
                    target_pose.pose.position.x, target_pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&WaypointNavigator::nav2_goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&WaypointNavigator::nav2_feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&WaypointNavigator::nav2_result_callback, this, _1);

        // Make sure Nav2 client is valid
        if (!nav2_action_client_)
        {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action client is not initialized!");
            auto result = std::make_shared<NavigateThroughWaypoints::Result>();
            result->success = false;
            result->message = "Internal error: Nav2 client invalid.";
            current_goal_handle_->abort(result);
            current_goal_handle_ = nullptr;
            current_navigation_plan_.clear();
            return;
        }

        nav2_action_client_->async_send_goal(nav2_goal_msg, send_goal_options);
    }

} // namespace waypoint_navigation

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_navigation::WaypointNavigator)
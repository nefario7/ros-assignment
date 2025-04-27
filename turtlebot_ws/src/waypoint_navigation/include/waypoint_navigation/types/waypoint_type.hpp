#ifndef WAYPOINT_NAVIGATION__TYPES__WAYPOINT_TYPE_HPP_
#define WAYPOINT_NAVIGATION__TYPES__WAYPOINT_TYPE_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace waypoint_navigation
{

    // Message type aliases
    using MsgPoint = geometry_msgs::msg::Point;
    using MsgPose = geometry_msgs::msg::Pose;
    using MsgPoseStamped = geometry_msgs::msg::PoseStamped;
    using MsgQuaternion = geometry_msgs::msg::Quaternion;
    using MsgTransformStamped = geometry_msgs::msg::TransformStamped;

    enum class WaypointType
    {
        MANUAL = 0,
        AUTOMATIC = 1,
    };

    struct Waypoint
    {
        std::string id;
        MsgPoseStamped pose;
        WaypointType type;
    };
} // namespace waypoint_navigation

#endif // WAYPOINT_NAVIGATION__TYPES__WAYPOINT_TYPE_HPP_

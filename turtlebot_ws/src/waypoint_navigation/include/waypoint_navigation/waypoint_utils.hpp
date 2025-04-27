#ifndef WAYPOINT_NAVIGATION__WAYPOINT_UTILS_HPP_
#define WAYPOINT_NAVIGATION__WAYPOINT_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <unordered_map>

#include "waypoint_navigation/types/waypoint_type.hpp"
#include "waypoint_navigation/impl/waypoint_utils_impl.hpp"

namespace waypoint_navigation
{

    double calculateDistance(const MsgPoint &p1, const MsgPoint &p2);
    double calculateDistance(const Waypoint &wp1, const Waypoint &wp2);

    double getYawFromQuaternion(const MsgQuaternion &q);
    double calculateYawDifference(double yaw1, double yaw2);

    std::unordered_map<std::string, Waypoint> loadWaypointsFromYaml(
        const std::string &file_path,
        const std::string &map_frame,
        rclcpp::Logger logger);

} // namespace waypoint_navigation

#endif // WAYPOINT_NAVIGATION__WAYPOINT_UTILS_HPP_

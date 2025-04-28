#ifndef WAYPOINT_NAVIGATION__PATH_PLANNER_HPP_
#define WAYPOINT_NAVIGATION__PATH_PLANNER_HPP_

#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <rclcpp/logging.hpp>

#include "waypoint_navigation/waypoint_manager.hpp" // For Waypoint struct

namespace waypoint_navigation
{
    //! @brief Plan the shortest path between two waypoints using Dijkstra's algorithm
    //! @param start_waypoint_id The ID of the starting waypoint
    //! @param end_waypoint_id The ID of the ending waypoint
    //! @param waypoints A map of waypoint IDs to their Waypoint structs
    //! @param connection_threshold The maximum distance between waypoints to consider them connected
    //! @param logger The logger to use for logging
    //! @return An optional vector of waypoint IDs representing the shortest path, or nullopt if no path exists
    std::optional<std::vector<std::string>> planShortestPath(
        const std::string &start_waypoint_id,
        const std::string &end_waypoint_id,
        const std::unordered_map<std::string, Waypoint> &waypoints,
        double connection_threshold,
        const rclcpp::Logger &logger);

} // namespace waypoint_navigation

#endif // WAYPOINT_NAVIGATION__PATH_PLANNER_HPP_
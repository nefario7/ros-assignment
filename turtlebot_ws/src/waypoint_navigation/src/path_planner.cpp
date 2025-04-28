#include "waypoint_navigation/path_planner.hpp"
#include "waypoint_navigation/waypoint_utils.hpp" // For calculateDistance

// Required for Dijkstra's
#include <queue>
#include <limits>
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>
#include <functional>
#include <algorithm>

namespace waypoint_navigation
{

    std::optional<std::vector<std::string>> planShortestPath(
        const std::string &start_waypoint_id,
        const std::string &end_waypoint_id,
        const std::unordered_map<std::string, Waypoint> &waypoints,
        double connection_threshold,
        const rclcpp::Logger &logger)
    {
        RCLCPP_INFO(logger, "Planning shortest path from '%s' to '%s'...",
                    start_waypoint_id.c_str(), end_waypoint_id.c_str());

        // Check if start and end waypoints exist
        if (waypoints.find(start_waypoint_id) == waypoints.end() ||
            waypoints.find(end_waypoint_id) == waypoints.end())
        {
            RCLCPP_ERROR(logger, "Start or end waypoint ID not found in provided waypoint map.");
            return std::nullopt;
        }

        // If start and end are the same, return a path with just the start waypoint
        if (start_waypoint_id == end_waypoint_id)
        {
            RCLCPP_INFO(logger, "Start and end waypoints are the same.");
            return std::vector<std::string>{start_waypoint_id};
        }

        // Build Adjacency List (weighted by distance)
        std::unordered_map<std::string, std::vector<std::pair<std::string, double>>> adj_list;
        for (const auto &pair1 : waypoints)
        {
            for (const auto &pair2 : waypoints)
            {
                if (pair1.first == pair2.first)
                    continue;

                double dist = calculateDistance(pair1.second, pair2.second);
                if (dist <= connection_threshold)
                {
                    adj_list[pair1.first].push_back({pair2.first, dist});
                }
            }
        }

        // Initialize distances and predecessors
        std::unordered_map<std::string, double> distances;
        std::unordered_map<std::string, std::string> predecessors;
        for (const auto &pair : waypoints)
        {
            distances[pair.first] = std::numeric_limits<double>::infinity();
        }
        distances[start_waypoint_id] = 0.0;

        // Priority Queue (Min-heap: distance, waypoint_id)
        using QueueElement = std::pair<double, std::string>;
        std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> pq;
        pq.push({0.0, start_waypoint_id});

        while (!pq.empty())
        {
            double d = pq.top().first;
            auto u = pq.top().second;
            pq.pop();

            // Skip if we found a shorter path already
            if (d > distances[u])
                continue;

            // If we reached the destination
            if (u == end_waypoint_id)
                break;

            // Explore neighbors
            if (adj_list.count(u))
            {
                for (const auto &edge : adj_list[u])
                {
                    std::string v = edge.first;
                    double weight = edge.second;
                    if (distances[u] + weight < distances[v])
                    {
                        distances[v] = distances[u] + weight;
                        predecessors[v] = u;
                        pq.push({distances[v], v});
                    }
                }
            }
        }

        // Reconstruct Path
        std::vector<std::string> path;
        std::string current = end_waypoint_id;
        if (distances[current] == std::numeric_limits<double>::infinity())
        {
            RCLCPP_ERROR(logger, "No path found from '%s' to '%s'.",
                         start_waypoint_id.c_str(), end_waypoint_id.c_str());
            return std::nullopt; // No path exists
        }

        while (current != start_waypoint_id)
        {
            path.push_back(current);
            if (predecessors.find(current) == predecessors.end())
            {
                RCLCPP_ERROR(logger, "Error reconstructing path. Predecessor not found for %s.", current.c_str());
                return std::nullopt;
            }
            current = predecessors[current];
        }

        path.push_back(start_waypoint_id);
        std::reverse(path.begin(), path.end());

        RCLCPP_INFO(logger, "Path found with %zu waypoints.", path.size());

        return path;
    }

} // namespace waypoint_navigation
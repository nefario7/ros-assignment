#include "waypoint_navigation/waypoint_utils.hpp"

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>

namespace waypoint_navigation
{

    double calculateDistance(const MsgPoint &p1, const MsgPoint &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) +
                         std::pow(p1.y - p2.y, 2));
    }

    double calculateDistance(const Waypoint &wp1, const Waypoint &wp2)
    {
        // Ignoring the rotation between the two waypoints
        return calculateDistance(wp1.pose.pose.position, wp2.pose.pose.position);
    }

    double getYawFromQuaternion(const MsgQuaternion &q_msg)
    {
        tf2::Quaternion tf2_q;
        // Use tf2::fromMsg to convert geometry_msgs::msg::Quaternion to tf2::Quaternion
        tf2::fromMsg(q_msg, tf2_q);
        tf2::Matrix3x3 m(tf2_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    double calculateYawDifference(double yaw1, double yaw2)
    {
        double diff = yaw1 - yaw2;
        // Normalize angle difference to [-pi, pi]
        while (diff > M_PI)
            diff -= 2.0 * M_PI;
        while (diff <= -M_PI)
            diff += 2.0 * M_PI;
        return diff;
    }

    std::unordered_map<std::string, Waypoint> loadWaypointsFromYaml(
        const std::string &file_path,
        const std::string &map_frame,
        rclcpp::Logger logger)
    {
        std::unordered_map<std::string, Waypoint> waypoints;
        RCLCPP_INFO(logger, "Loading waypoints from %s...", file_path.c_str());

        try
        {
            YAML::Node data = YAML::LoadFile(file_path);
            if (!data["waypoints"])
            {
                RCLCPP_WARN(logger, "Waypoint file [%s] does not contain 'waypoints' key.", file_path.c_str());
                return waypoints;
            }

            for (const auto &node : data["waypoints"])
            {
                Waypoint wp;
                wp.id = node["id"].as<std::string>();
                wp.pose.header.frame_id = node["frame_id"].as<std::string>(map_frame);

                // Position
                auto &pos = node["pose"]["position"];
                wp.pose.pose.position.x = pos["x"].as<double>();
                wp.pose.pose.position.y = pos["y"].as<double>();
                wp.pose.pose.position.z = pos["z"].as<double>(0.0);

                // Orientation (default to identity quaternion)
                auto &orient = node["pose"]["orientation"];
                wp.pose.pose.orientation.x = orient["x"].as<double>(0.0);
                wp.pose.pose.orientation.y = orient["y"].as<double>(0.0);
                wp.pose.pose.orientation.z = orient["z"].as<double>(0.0);
                wp.pose.pose.orientation.w = orient["w"].as<double>(1.0);

                // Type
                wp.type = (node["type"].as<std::string>("AUTOMATIC") == "MANUAL")
                              ? WaypointType::MANUAL
                              : WaypointType::AUTOMATIC;

                // Validation
                if (wp.pose.header.frame_id != map_frame)
                {
                    RCLCPP_WARN(logger, "Waypoint '%s' has frame '%s' but expected '%s'.",
                                wp.id.c_str(), wp.pose.header.frame_id.c_str(), map_frame.c_str());
                }

                if (waypoints.count(wp.id))
                {
                    RCLCPP_WARN(logger, "Duplicate waypoint ID '%s' found in file. Overwriting.", wp.id.c_str());
                }

                waypoints[wp.id] = wp;
            }
            RCLCPP_INFO(logger, "Successfully loaded %zu waypoints.", waypoints.size());
        }
        catch (const YAML::BadFile &e)
        {
            RCLCPP_WARN(logger, "Waypoint file [%s] not found or invalid. Error: %s", file_path.c_str(), e.what());
            RCLCPP_INFO(logger, "Starting with no waypoints. New waypoints will be saved to this location.");
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(logger, "Failed to parse waypoint file [%s]. Error: %s", file_path.c_str(), e.what());
            RCLCPP_INFO(logger, "Starting with no waypoints. New waypoints will be saved to this location.");
        }

        return waypoints;
    }

} // namespace waypoint_navigation

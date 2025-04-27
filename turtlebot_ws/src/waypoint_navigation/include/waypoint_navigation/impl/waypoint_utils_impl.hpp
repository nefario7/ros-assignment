#ifndef WAYPOINT_NAVIGATION__IMPL__WAYPOINT_UTILS_IMPL_HPP_
#define WAYPOINT_NAVIGATION__IMPL__WAYPOINT_UTILS_IMPL_HPP_

// This file is included by waypoint_utils.hpp
// Do not include it directly

namespace waypoint_navigation
{

    template <typename T>
    T declare_parameter_if_not_declared(
        rclcpp::Node *node,
        const std::string &name,
        const T &default_value,
        const rcl_interfaces::msg::ParameterDescriptor &descriptor)
    {
        if (!node->has_parameter(name))
        {
            return node->declare_parameter(name, default_value, descriptor);
        }
        // Use get_parameter_or to handle cases where parameter exists but has wrong type?
        // For now, assume type is correct if parameter exists.
        return node->get_parameter(name).get_value<T>();
    }

} // namespace waypoint_navigation

#endif // WAYPOINT_NAVIGATION__IMPL__WAYPOINT_UTILS_IMPL_HPP_
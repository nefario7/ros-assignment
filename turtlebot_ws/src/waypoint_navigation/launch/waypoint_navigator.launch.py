import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_name = 'waypoint_navigation'
    pkg_share = get_package_share_directory(pkg_name)

    params_file = os.path.join(pkg_share, 'config', 'waypoint_navigator_params.yaml')

    navigator_node = ComposableNode(
        package=pkg_name,
        plugin='waypoint_navigation::WaypointNavigator', 
        name='waypoint_navigator',                   
        parameters=[params_file],
        extra_arguments=[{'use_intra_process_comms': True}] 
    )
    

    container = ComposableNodeContainer(
        name='waypoint_nav_nav_container', 
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[navigator_node],
        output='screen',
    )

    return LaunchDescription([
        container
    ]) 
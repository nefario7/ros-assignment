import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_name = 'waypoint_navigation'
    pkg_share = get_package_share_directory(pkg_name)
    nav_params_file = os.path.join(pkg_share, 'config', 'nav_params.yaml')
    # rviz_config_file = os.path.join(pkg_share, 'config', 'nav2_waypoints.rviz')

    # Set TURTLEBOT3_MODEL environment variable
    turtlebot_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    set_turtlebot_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot_model)

    # Simulation (TurtleBot3 Example)
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # Navigation Stack (Nav2 for TurtleBot3 Example)
    pkg_nav2_turtlebot3 = get_package_share_directory('turtlebot3_navigation2')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_turtlebot3, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True', 
                          'params_file': os.path.join(pkg_nav2_turtlebot3, 'param', f'{turtlebot_model}.yaml'), 
                          'autostart': 'True',
                          'use_rviz': 'False',
                          'slam': 'True'}.items()
    )

    # Waypoint Manager Node Component
    waypoint_manager_node = ComposableNode(
        package=pkg_name,
        plugin='waypoint_navigation::WaypointManager',
        name='waypoint_manager',
        parameters=[nav_params_file],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Waypoint Navigator Node Component
    waypoint_navigator_node = ComposableNode(
        package=pkg_name,
        plugin='waypoint_navigation::WaypointNavigator',
        name='waypoint_navigator',
        parameters=[nav_params_file],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Composable Node Container that holds both components
    container = ComposableNodeContainer(
        name='waypoint_nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            waypoint_manager_node,
            waypoint_navigator_node
        ],
        output='screen',
    )

    return LaunchDescription([
        set_turtlebot_model,
        simulation_launch,
        nav2_launch, 
        container, 
    ]) 
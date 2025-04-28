# ROS Anscer Assignment

This repository contains a ROS 2 package (`waypoint_navigation`) for managing and navigating waypoints with a TurtleBot3 (or similar robot) in a simulated environment.

### Reference :
- Environment [sea-bass/turtlebot3_behavior_demos](https://github.com/sea-bass/turtlebot3_behavior_demos)

## `waypoint_navigation` Package Implementation Details

The core functionality is provided by two ROS 2 nodes:

1.  **`WaypointManager` (`waypoint_manager`)**:
    *   Responsible for creating, storing, loading, and saving waypoints.
    *   Supports manual waypoint creation via the service `~/create_waypoint` service.
    *   Supports automatic waypoint creation based on distance and angle thresholds travelled (monitored via `/odom` and handled with `odomCallback()`).
    *   Loads and saves waypoints to/from a specified YAML file (parameter `waypoint_file_path`).
    *   Publishes visualization markers (`~/waypoints_viz`) for saved waypoints which can be visualized in Rviz2. These are colored depending on how they were generated, blue for automatic and green for manual.

2.  **`WaypointNavigator` (`waypoint_navigator`)**:
    *   Receives navigation goals (specific waypoint names) via the `/navigate_to_waypoint` action server.
    *   Loads waypoints from the same YAML file used by the `WaypointManager`.
    *   Uses the Nav2 `navigate_to_pose` action client to send the robot to the target waypoint pose.
    *   Determines the closest waypoint to the robot's current position to start navigation or check proximity.
    *   Connects waypoints based on proximity (parameter `waypoint_connection_threshold`) to potentially represent a graph structure (though full graph navigation isn't implemented here).


**Build and Run:**

1.  **Build the Workspace:**
    Navigate to the root of your ROS 2 workspace (`turtlebot_ws`) and build the package:
    ```bash
    cd /path/to/the/turtlebot_ws 
    colcon build --packages-select waypoint_navigation
    OR
    colcon build # This will build all packages including the original ones from the environment.
    ```

2.  **Source the Workspace:**
    Before running any nodes, source the setup file in your current terminal:
    ```bash
    source /opt/ros/jazzy/setup.zsh
    source install/setup.zsh 
    ```

3.  **Launch the Nodes and Simulation:**
    (Terminal 0) Launch the TurtleBot3 Gazebo simulation environment (ensure you have the necessary packages from the reference environment):
    ```bash
    ros2 launch waypoint_navigation navigate_waypoints.launch.py
    ```

6.  **Interact:**
    *   Terminal 1 : Use the service call to create manual waypoints. Example:  
        The manual waypoints can be generated using the following command where the `<point_id>` refers to the ID of the waypoint. 
        ```bash
        ros2 service call /waypoint_manager/create_waypoint waypoint_navigation/srv/CreateWaypoint "{waypoint_id: '<point_id>'}"
        ```

    *   Terminal 2 : Drive the robot around (e.g., using `teleop_keyboard`) to trigger automatic waypoint creation. (Open in a new terminal after sourcing the workspaces)
        ```bash
        ros2 run turtlebot3_teleop teleop_keyboard 
        ```
    *   Terminal 3 : Use action client to send navigation goals to `/navigate_to_waypoint`. Example:
        ```bash
        # Replace 'waypoint_1' with the desired waypoint ID from your waypoints.yaml
        ros2 action send_goal /navigate_to_waypoint waypoint_navigation/action/NavigateToWaypoint "{waypoint_id: 'waypoint_1'}" 
        ``` 
7. **Rviz:**
    * Rviz by default will be missing the Markers visualization, add the visualization by adding the `/waypoint_manager/waypoints_viz` topic to the display.
    * You can turn of the `Controller` and `Global Planner` for cleaner visualization.
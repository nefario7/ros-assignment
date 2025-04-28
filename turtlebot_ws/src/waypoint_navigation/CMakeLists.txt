cmake_minimum_required(VERSION 3.10)
project(waypoint_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED) # Added PkgConfig
find_package(rosidl_default_generators REQUIRED) # Added for service generation
find_package(rcl_interfaces REQUIRED)

# Use pkg-config to find yaml-cpp (more reliable)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# Add service definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CreateWaypoint.srv"
  "action/NavigateThroughWaypoints.action"
)

# List all source files for the shared library
set(WAYPOINT_LIB_SOURCES
  src/waypoint_manager.cpp
  src/waypoint_navigator.cpp
  src/waypoint_utils.cpp
  src/path_planner.cpp
)

# Create a single shared library for all components
add_library(waypoint_navigation_lib SHARED ${WAYPOINT_LIB_SOURCES})

target_include_directories(waypoint_navigation_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp> # Include for generated headers
  $<INSTALL_INTERFACE:include/rosidl_generator_cpp> # Include for generated headers (install)
  ${YAML_CPP_INCLUDE_DIRS} # Add yaml-cpp include directories
)
target_compile_features(waypoint_navigation_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Register the components from the library
rclcpp_components_register_nodes(waypoint_navigation_lib "waypoint_navigation::WaypointManager")
rclcpp_components_register_nodes(waypoint_navigation_lib "waypoint_navigation::WaypointNavigator")

ament_target_dependencies(
  waypoint_navigation_lib
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "visualization_msgs"
  "nav_msgs"
  "nav2_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "rcl_interfaces"
)

# Link library against generated C++ typesupport library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(waypoint_navigation_lib "${cpp_typesupport_target}")

# Also link library against C typesupport if necessary (often required indirectly)
rosidl_get_typesupport_target(c_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_c")
target_link_libraries(waypoint_navigation_lib "${c_typesupport_target}")

# Explicitly link library against yaml-cpp library using pkg-config results
target_link_libraries(waypoint_navigation_lib ${YAML_CPP_LIBRARIES})

# Install the library
install(TARGETS waypoint_navigation_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install component registration info (can be generated automatically now)
# install(
#     FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/node_components.xml
#     DESTINATION share/${PROJECT_NAME}/cmake
# )

# Install service and action definitions
install(
  DIRECTORY srv action
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config file(s)
install(
  FILES 
    config/nav_params.yaml # Install combined params
    config/nav2_waypoints.rviz # Keep Rviz config install
  DESTINATION share/${PROJECT_NAME}/config # Destination directory
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

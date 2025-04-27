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
)

# List all source files needed for the waypoint_manager library
set(WAYPOINT_MANAGER_SOURCES
  src/waypoint_manager.cpp
  src/waypoint_utils.cpp 
)

# Changed from add_executable to add_library for component node
add_library(waypoint_manager SHARED ${WAYPOINT_MANAGER_SOURCES})

target_include_directories(waypoint_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp> # Include for generated headers
  $<INSTALL_INTERFACE:include/rosidl_generator_cpp> # Include for generated headers (install)
  ${YAML_CPP_INCLUDE_DIRS} # Add yaml-cpp include directories
)
target_compile_features(waypoint_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Register the component
rclcpp_components_register_nodes(waypoint_manager "waypoint_navigation::WaypointManager")

ament_target_dependencies(
  waypoint_manager
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
  "rcl_interfaces" # Added rcl_interfaces
)

# Link against generated C++ typesupport library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(waypoint_manager "${cpp_typesupport_target}")

# Also link against C typesupport if necessary (often required indirectly)
rosidl_get_typesupport_target(c_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_c")
target_link_libraries(waypoint_manager "${c_typesupport_target}")

# Explicitly link yaml-cpp library using pkg-config results
target_link_libraries(waypoint_manager ${YAML_CPP_LIBRARIES})

# Install the library, not the executable
install(TARGETS waypoint_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install component registration info - Temporarily disabled
# install(
#     FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/node_components.xml
#     DESTINATION share/${PROJECT_NAME}/cmake
# )

# Install service definition
install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config file(s)
install(
  FILES config/waypoint_params.yaml # Install specific file
  DESTINATION share/${PROJECT_NAME}/config # Destination directory
)

# Install Rviz config file
install(
  FILES config/nav2_waypoints.rviz
  DESTINATION share/${PROJECT_NAME}/config
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

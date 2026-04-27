cmake_minimum_required(VERSION 3.8)
project(bme_gazebo_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(goto_server SHARED
  src/goto_server.cpp)
target_include_directories(goto_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(goto_server
  PRIVATE "GOTO_SERVER_BUILDING_DLL")
ament_target_dependencies(goto_server
  action_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_msgs  
  geometry_msgs
  tf2 
  tf2_ros
  tf2_geometry_msgs
  )
rclcpp_components_register_node(goto_server PLUGIN "GoToServer" EXECUTABLE goto_server_node)
install(TARGETS
  goto_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(goto_client SHARED
  src/goto_client.cpp)
target_include_directories(goto_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(goto_client
  PRIVATE "GOTO_SERVER_BUILDING_DLL")
ament_target_dependencies(goto_client
  rclcpp
  action_interfaces
  rclcpp_action
  geometry_msgs
  rclcpp_components
)
rclcpp_components_register_node(goto_client PLUGIN "GoToClient" EXECUTABLE goto_client_node)
install(TARGETS
  goto_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


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

install(DIRECTORY
  launch
  worlds
  rviz
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

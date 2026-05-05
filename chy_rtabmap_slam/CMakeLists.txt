cmake_minimum_required(VERSION 3.8)
project(chy_rtabmap_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ardupilot_msgs REQUIRED)  # APM 消息包
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(rtabmap_slam_node src/rtabmap_slam_node.cpp)
target_include_directories(rtabmap_slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rtabmap_slam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  rtabmap_slam_node
  "rclcpp"
)

install(TARGETS rtabmap_slam_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(right_cam_info_fix src/right_cam_info_fix.cpp)
ament_target_dependencies(right_cam_info_fix rclcpp sensor_msgs)

install(TARGETS right_cam_info_fix
        DESTINATION lib/${PROJECT_NAME})
        





add_executable(vel_converter_node src/vel_converter.cpp)
ament_target_dependencies(vel_converter_node rclcpp geometry_msgs)

target_include_directories(vel_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(vel_converter_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS vel_converter_node
  DESTINATION lib/${PROJECT_NAME})







add_executable(map_pub src/map_pub.cpp)
ament_target_dependencies(map_pub rclcpp nav_msgs)

install(TARGETS
  map_pub
  DESTINATION lib/${PROJECT_NAME})







install(DIRECTORY include/
    DESTINATION include
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY map
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
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

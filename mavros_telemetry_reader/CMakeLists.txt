cmake_minimum_required(VERSION 3.8)
project(mavros_telemetry_reader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros REQUIRED)   # 名字必须是 mavros
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2    REQUIRED)   # tf2_ros 会间接用到，也写上更保险
find_package(pluginlib REQUIRED)
find_package(libmavconn REQUIRED)

add_executable(telemetry_reader src/telemetry_reader.cpp)
target_include_directories(telemetry_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(telemetry_reader PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(telemetry_reader
  rclcpp mavros_msgs sensor_msgs geometry_msgs)


add_executable(mavros_node src/chy_mavros_node.cpp)
target_include_directories(mavros_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mavros_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(mavros_node
  rclcpp mavros_msgs sensor_msgs geometry_msgs mavros diagnostic_msgs tf2_ros tf2 pluginlib libmavconn)


add_executable(chy_control src/chy_control.cpp)
ament_target_dependencies(chy_control rclcpp mavros_msgs)


install(TARGETS telemetry_reader mavros_node chy_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
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

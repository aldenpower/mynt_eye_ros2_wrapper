cmake_minimum_required(VERSION 3.20)

project(mynt_eye_ros2_wrapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(mynteye REQUIRED)



# Node executable
add_executable(${PROJECT_NAME} src/wrapper_node.cc)

target_include_directories(${PROJECT_NAME}
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  # ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  # cv_bridge::cv_bridge
  image_transport::image_transport
  # sensor_msgs::sensor_msgs_library
  # tf2::tf2
  # tf2_ros::static_transform_broadcaster_node
  # tf2_ros::tf2_ros
  mynteye
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
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

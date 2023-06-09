cmake_minimum_required(VERSION 3.5)
project(slam)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package (glog REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
  include/slam
  include/ORB_SLAM3
  include/ORB_SLAM3/include
  include/ORB_SLAM3/include/CameraModels
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(
  include/ORB_SLAM3/lib
)
add_executable(mono_slam src/mono_slam.cpp)
add_executable(ros_mono src/mono_slam_ros.cpp)
add_executable(ros_rgbd src/rgbd_slam_ros.cpp)
ament_target_dependencies(mono_slam
  rclcpp 
)
ament_target_dependencies(ros_mono
  rclcpp std_msgs sensor_msgs cv_bridge
)
ament_target_dependencies(ros_rgbd
  rclcpp std_msgs sensor_msgs cv_bridge
)
target_link_libraries(mono_slam
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ORB_SLAM3
)
target_link_libraries(ros_mono
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ORB_SLAM3
)
target_link_libraries(ros_rgbd
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  ORB_SLAM3
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
install(
  TARGETS 
  mono_slam
  ros_mono
  ros_rgbd
  DESTINATION
  lib/${PROJECT_NAME}
)
install(TARGETS ros_mono
  DESTINATION
  lib/${PROJECT_NAME}
)


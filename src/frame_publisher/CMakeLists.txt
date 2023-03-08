cmake_minimum_required(VERSION 3.5)
project(frame_publisher)
set(CMAKE_BUILD_TYPE "Debug")
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(glog REQUIRED)
#OpenCV
find_package(OpenCV REQUIRED)
# set(OpenCV_INCLUDE_DIRS )
# message(${OpenCV_LIBS})
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include/frame_publisher
  include/OrbbecSDK/include
)
link_directories(
  include/OrbbecSDK/lib
)
add_executable(publisher src/opencv_frame_publisher.cpp)
add_executable(publisher_obsdk src/ob_frame_publisher.cpp)
add_executable(listener src/opencv_frame_listener.cpp)
ament_target_dependencies(publisher
  rclcpp std_msgs cv_bridge sensor_msgs
)

target_link_libraries(publisher
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_videoio
)

target_link_libraries(publisher_obsdk
  OrbbecSDK
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_videoio
)
ament_target_dependencies(publisher_obsdk
  rclcpp std_msgs cv_bridge sensor_msgs
)
ament_target_dependencies(listener
  rclcpp std_msgs cv_bridge sensor_msgs
)
install(TARGETS 
  publisher_obsdk
  DESTINATION
  lib/${PROJECT_NAME}
)
install(TARGETS 
  listener
  publisher
  
  DESTINATION
  lib/${PROJECT_NAME}
)
ament_package()

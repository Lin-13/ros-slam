cmake_minimum_required(VERSION 3.5)
project(frame_publisher)

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
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(publisher src/opencv_frame_publisher.cpp)
add_executable(listener src/opencv_frame_listener.cpp)
ament_target_dependencies(publisher
  rclcpp std_msgs
)
ament_target_dependencies(listener
  rclcpp std_msgs
)
target_link_libraries(publisher
  ${OpenCV_LIBS}
)
install(TARGETS 
  publisher
  DESTINATION
  lib/${PROJECT_NAME}
)
install(TARGETS 
  listener
  DESTINATION
  lib/${PROJECT_NAME}
)
ament_package()

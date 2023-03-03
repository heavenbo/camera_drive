cmake_minimum_required(VERSION 3.6)
project(detect)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    roslib
    # msg
    std_msgs
    sensor_msgs
)
include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

add_executable(opencv src/opencv.cpp)
target_link_libraries(opencv ${OpenCV_LIBS} ${catkin_LIBRARIES} gxiapi -lpthread -lm)

set(CMAKE_BUILD_TYPE Release)
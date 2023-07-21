cmake_minimum_required(VERSION 3.11)
project(gxcam)

set(CMAKE_CXX_STANDARD 11)


add_executable(debug src/debug.cpp)
target_link_libraries(debug  gxiapi yaml-cpp -lpthread -lm)   
# add_executable(save src/save.cpp)
# target_link_libraries(save ${OpenCV_LIBS} ${catkin_LIBRARIES})                    

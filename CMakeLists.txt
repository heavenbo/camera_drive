cmake_minimum_required(VERSION 3.11)
project(detect)

set(CMAKE_CXX_STANDARD 11)

find_package(CUDA  REQUIRED)
 

find_package(OpenCV 4 REQUIRED)
find_package(vpi 2.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    # tf
    roscpp
    roslib
    mavros_msgs
    std_msgs
    sensor_msgs
)
find_package(jetson-utils REQUIRED)

set(ENABLE_NVMM_DEFAULT ON)
include_directories(/usr/src/jetson_multimedia_api/include /opt/ros/noetic/include /usr/include/libdrm)
include_directories("${CUDA_INCLUDE_DIRS}" ~/acfly_ws/devel/include/ /usr/include/opencv4/)
 

add_library(Gxdrive 
            include/Gxdrive2.h
            include/Gxdrive2.cpp
        )
# set(SOURCES

# src/test_CUDA.cpp
# /usr/src/jetson_multimedia_api/argus/samples/cudaBayerDemosaic/CudaBayerDemosaicKernel.cu
# )
link_directories(/usr/lib/aarch64-linux-gnu/tegra)
link_directories(/usr/local/cuda/lib64)
link_directories(/usr/src/jetson_multimedia_api/samples/common/classes)
find_library(NVBUFF_LIB REQUIRED NAMES nvbuf_utils HINTS /usr/lib/aarch64-linux-gnu/tegra )
message("nvbuf_utils:${NVBUFF_LIB}")
cuda_add_executable(opencv src/Drive2.cpp)
target_link_libraries(opencv  Gxdrive ${NVBUFF_LIB} ${OpenCV_LIBS} ${catkin_LIBRARIES} 
								vpi yaml-cpp gxiapi -lv4l2 -lEGL -lGLESv2 -lX11 -lnvbufsurface 
								-lnvbufsurftransform -ldrm -lcuda -lcudart jetson-utils -lnppc -lnppicc
                                -lnppidei jetson-utils -lnppif -lnppial -lnppim -lnppist -lnppisu -lnppitc -lnpps
                                -lnppig)

set(CMAKE_BUILD_TYPE Release)

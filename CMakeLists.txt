cmake_minimum_required(VERSION 3.16)
project(v1)

set(PCL_INCLUDE_DIRS  "/usr/local/include/pcl-1.13")

find_package(k4a REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


include_directories(.)
include_directories( 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
# OpenCV_LIBS和PCL_LIBRARIES，是否缩写是有区别的
aux_source_directory(./src SRC_LIST)
add_executable(v1 src/camera.cpp src/pointcloud.cpp)

target_link_libraries(v1 k4a::k4a)
target_link_libraries(v1 
    ${OpenCV_LIBS}   
    ${PCL_LIBRARIES}
)
# 查看点云
project(v2)

set(PCL_INCLUDE_DIRS  "/usr/local/include/pcl-1.13")

find_package(k4a REQUIRED)
find_package(PCL REQUIRED)

include_directories(.)
include_directories( 
    ${PCL_INCLUDE_DIRS}
)
add_executable(v2 src/seePLY.cpp)
target_link_libraries(v2 k4a::k4a)
target_link_libraries(v2 
    ${PCL_LIBRARIES}
)
# # 点云配准
project(v3)

set(PCL_INCLUDE_DIRS  "/usr/local/include/pcl-1.13")

find_package(k4a REQUIRED)
find_package(PCL REQUIRED)

include_directories(.)
include_directories( 
    ${PCL_INCLUDE_DIRS}
)
add_executable(v3 src/CloudCompare.cpp )
target_link_libraries(v3 k4a::k4a)
target_link_libraries(v3 
    ${PCL_LIBRARIES}
)
project(v4)

set(PCL_INCLUDE_DIRS  "/usr/local/include/pcl-1.13")

find_package(k4a REQUIRED)
find_package(PCL REQUIRED)

include_directories(.)
include_directories( 
    ${PCL_INCLUDE_DIRS}
)
add_executable(v4 src/twoPointCloudCompare.cpp )
target_link_libraries(v4 k4a::k4a)
target_link_libraries(v4 
    ${PCL_LIBRARIES}
)
project(v5)

set(PCL_INCLUDE_DIRS  "/usr/local/include/pcl-1.13")

find_package(k4a REQUIRED)
find_package(PCL REQUIRED)

include_directories(.)
include_directories( 
    ${PCL_INCLUDE_DIRS}
)
add_executable(v5 src/pclNormalization.cpp )
target_link_libraries(v5 k4a::k4a)
target_link_libraries(v5 
    ${PCL_LIBRARIES}
)

# project(v1.1)

# set(PCL_INCLUDE_DIRS  "/usr/local/include/pcl-1.13")

# find_package(k4a REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(PCL REQUIRED)


# include_directories(.)
# include_directories( 
#     ${PCL_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
# )
# # OpenCV_LIBS和PCL_LIBRARIES，是否缩写是有区别的
# aux_source_directory(./src SRC_LIST)
# add_executable(v1.1 src/camera.cpp src/ros.cpp)

# target_link_libraries(v1.1 k4a::k4a)
# target_link_libraries(v1.1 
#     ${OpenCV_LIBS}   
#     ${PCL_LIBRARIES}
# )
cmake_minimum_required(VERSION 3.10)
project(PCDToGridMap VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 仅依赖 PCL（含 segmentation）
find_package(PCL 1.10 REQUIRED COMPONENTS common io filters segmentation)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 唯一可执行文件
add_executable(pcd_to_gridmap pcd_to_gridmap.cpp)
target_link_libraries(pcd_to_gridmap ${PCL_LIBRARIES})
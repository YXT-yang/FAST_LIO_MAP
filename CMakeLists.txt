cmake_minimum_required(VERSION 3.5)
project(fast-lio-map)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# solve pcl warning
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)   # 4.2.0
find_package(PCL REQUIRED)      # 1.10.0
find_package(Threads)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/ikd-Tree IKD-TREE)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/include/map-Maintain MAP-MAINTAIN)

add_library(${PROJECT_NAME}_lib 
    ${IKD-TREE}
    ${MAP-MAINTAIN}
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}_lib 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    Threads::Threads
)

add_executable(${PROJECT_NAME}_node 
    exe/main.cpp
)

add_executable(generate_point
    exe/generate_point.cpp
)
add_executable(generate_point_add
    exe/generate_point_add.cpp
)

target_link_libraries(${PROJECT_NAME}_node 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_lib
)

target_link_libraries(generate_point
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_lib
)
target_link_libraries(generate_point_add
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}_lib
)
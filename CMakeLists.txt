cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_visualizer pcl_visualizer.cpp)
target_link_libraries(pcl_visualizer ${PCL_LIBRARIES})

add_executable(statOutlierVisualize statOutlierVisualize.cpp)
target_link_libraries(statOutlierVisualize ${PCL_LIBRARIES})

add_executable(statOutlierBundled statOutlierBundled.cpp)
target_link_libraries(statOutlierBundled ${PCL_LIBRARIES})

add_executable(incrementalRegister incrementalRegister.cpp)
target_link_libraries(incrementalRegister ${PCL_LIBRARIES})

add_executable(incrementalRegister_noVis incrementalRegister_noVis.cpp)
target_link_libraries(incrementalRegister_noVis ${PCL_LIBRARIES})

add_executable(concatenateClouds concatenateClouds.cpp)
target_link_libraries(concatenateClouds ${PCL_LIBRARIES})

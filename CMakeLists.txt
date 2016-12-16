cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)

set(OpenCV_DIR "/usr/include/opencv2")

find_package(PCL 1.3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#list(APPEND LIBRARIES_LIST ${OpenCV_LIBRARIES} )

#add_executable(loadImage loadImage.cpp)
#target_link_libraries(loadImage ${OpenCV_LIBS})

#add_executable(image_brightness image_brightness.cpp)
#target_link_libraries(image_brightness ${OpenCV_LIBS})

#add_executable(Flann Flann.cpp)
#target_link_libraries(Flann ${OpenCV_LIBS} opencv_nonfree)

#add_executable(BruteForce BruteForce.cpp)
#target_link_libraries(BruteForce ${OpenCV_LIBS} opencv_nonfree)

include_directories(include)
set(SOURCES src/SGBM4.cpp src/write_ply.cpp src/scale_image.cpp src/feature_match.cpp)# ${include}/write_ply.h)

add_executable(SGBM SGBM.cpp)
target_link_libraries(SGBM ${OpenCV_LIBS} opencv_nonfree)

add_executable(SGBM2 SGBM2.cpp)
target_link_libraries(SGBM2 ${OpenCV_LIBS} opencv_nonfree)

add_executable(SGBM3 SGBM3.cpp)
target_link_libraries(SGBM3 ${OpenCV_LIBS} opencv_nonfree)

add_executable(SGBM4 ${SOURCES})
target_link_libraries(SGBM4 ${OpenCV_LIBS} opencv_nonfree)

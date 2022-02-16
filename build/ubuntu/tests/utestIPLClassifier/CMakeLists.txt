cmake_minimum_required( VERSION 2.8 )

project(utests)
set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories("/usr/include/eigen3")
include_directories("/home/appx/Appropolis/code/c/ipl/include/")


find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(test_ipl_classifier utest_iplclassifier/utest_iplclassifier.cpp ../../modules/core/iplfiles.cpp ../../modules/core/iplstring.cpp ../../modules/classifier/classify_wall_ceiling.cpp ../../modules/io/pointcloud/pointcloud_io.cpp ../../include/io/impl/pointcloud/pointcloud_io.hpp)
target_link_libraries(test_ipl_classifier ${PCL_LIBRARIES})

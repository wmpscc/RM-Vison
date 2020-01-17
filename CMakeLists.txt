cmake_minimum_required(VERSION 3.9)
project(RM_Armor)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(RM_Armor main.cpp find_armor.h find_light_blobs.cpp constants.h find_armor_box.cpp show_images.cpp show_images.h find_armor.cpp)

target_link_libraries(RM_Armor ${OpenCV_LIBS})
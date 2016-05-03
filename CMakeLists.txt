cmake_minimum_required(VERSION 2.8.3)
project(open_rm)

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

include_directories(include ${Boost_INCLUDE_DIRECTORIES} ${OpenCV_INCLUDE_DIRECTORIES})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES open_rm
  DEPENDS OpenCV Boost
)

add_library(vision include/open_rm/Vision/calibrage.h src/Vision/calibrage.cpp)
target_link_libraries(vision ${OpenCV_LIBRARIES})

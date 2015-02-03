cmake_minimum_required(VERSION 2.8.3)
project(rose_twist_moving_average_filter)

find_package(catkin REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
  	INCLUDE_DIRS 
  		include
  	LIBRARIES 
  		twist_moving_average_filter
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

add_library(twist_moving_average_filter 
	SHARED
	src/twist_maf.cpp
)

target_link_libraries(twist_moving_average_filter ${catkin_LIBRARIES})
add_dependencies(twist_moving_average_filter ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
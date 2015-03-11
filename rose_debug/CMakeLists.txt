cmake_minimum_required(VERSION 2.8.3)
project(rose_debug)

find_package(catkin REQUIRED
	COMPONENTS
		roscpp
	)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS 
		include
    LIBRARIES 
		rose_debug 
	CATKIN_DEPENDS
		roscpp
	DEPENDS
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

add_library(rose_debug 
	SHARED
	src/rose_timing.cpp
)

target_link_libraries(rose_debug ${catkin_LIBRARIES})
add_dependencies(rose_debug ${PROJECT_NAME}_generate_messages_cpp)

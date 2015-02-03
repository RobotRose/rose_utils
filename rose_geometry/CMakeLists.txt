cmake_minimum_required(VERSION 2.8.3)
project(rose_geometry)

find_package(catkin REQUIRED
	COMPONENTS
		geometry_msgs
		rose_common
	)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS 
		include
    LIBRARIES 
		rose_geometry
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

add_library(rose_geometry 
	SHARED
	src/point.cpp
	src/stamped.cpp
	# std/geomtry.cpp
)

target_link_libraries(rose_geometry ${catkin_LIBRARIES})
add_dependencies(rose_geometry ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

cmake_minimum_required(VERSION 2.8.3)
project(rose_transformations)

find_package(catkin REQUIRED
	COMPONENTS
		geometry_msgs
		rose_conversions
		rose_common
		rose_geometry
		tf
	)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS 
		include
    LIBRARIES 
		rose_transformations
	CATKIN_DEPENDS
		rose_geometry 
		tf
	DEPENDS
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

add_library(rose_transformations 
	SHARED
	src/transformations.cpp
	src/tf_helper.cpp
)

target_link_libraries(rose_transformations ${catkin_LIBRARIES})
add_dependencies(rose_transformations ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

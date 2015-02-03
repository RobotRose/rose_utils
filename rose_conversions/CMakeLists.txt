cmake_minimum_required(VERSION 2.8.3)
project(rose_conversions)

find_package(catkin REQUIRED
	COMPONENTS
		geometry_msgs
	)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS 
		include
    LIBRARIES 
		rose_conversions
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

add_library(rose_conversions 
	SHARED
	src/conversions.cpp
)

target_link_libraries(rose_conversions ${catkin_LIBRARIES})
add_dependencies(rose_conversions ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

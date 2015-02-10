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
	CATKIN_DEPENDS
		geometry_msgs
		rose_common
	DEPENDS
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

add_library(rose_geometry 
	SHARED
	src/point.cpp
	src/stamped.cpp
	src/geometry.cpp
)

target_link_libraries(rose_geometry ${catkin_LIBRARIES})
add_dependencies(rose_geometry ${PROJECT_NAME}_generate_messages_cpp)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-utest unit_tests/geometry_unit_test.cpp)
if(TARGET ${PROJECT_NAME}-utest)
  target_link_libraries(${PROJECT_NAME}-utest ${PROJECT_NAME})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

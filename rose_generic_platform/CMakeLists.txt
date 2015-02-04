cmake_minimum_required(VERSION 2.8.3)
project(rose_generic_platform)

find_package(catkin REQUIRED
	COMPONENTS
		opteq_wheelunits_01
		rose_common
	)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS 
		include
    LIBRARIES 
		rose_generic_platform
	CATKIN_DEPENDS
		opteq_wheelunits_01
		rose_common 
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

add_library(rose_generic_platform 
	SHARED
	src/generic_platform.cpp
)

target_link_libraries(rose_generic_platform ${catkin_LIBRARIES})
add_dependencies(rose_generic_platform ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

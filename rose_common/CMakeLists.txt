cmake_minimum_required(VERSION 2.8.3)
project(rose_common)

find_package(catkin REQUIRED
	#COMPONENTS
)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS 
		include
    # LIBRARIES 
		# rose_common
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

add_library(rose_common 
	SHARED
	src/PID.cpp
	src/common.cpp
	src/backward.cpp	
)

target_link_libraries(rose_common ${catkin_LIBRARIES})
add_dependencies(rose_common ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
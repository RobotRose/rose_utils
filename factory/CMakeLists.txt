cmake_minimum_required(VERSION 2.8.3)
project(factory)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC")

find_package(catkin REQUIRED
	COMPONENTS
	luctor_classes
	pcl_conversions 
	pcl_ros 
	roscomm
)

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES factory
  CATKIN_DEPENDS roscomm luctor_classes  
  DEPENDS pcl_conversions pcl_ros 
)

add_library(factory SHARED src/bounding_box_factory/bounding_box_factory.cpp )
target_link_libraries(factory ${catkin_LIBRARIES})

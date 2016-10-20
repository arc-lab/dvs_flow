cmake_minimum_required(VERSION 2.8.3)
project(dvs_flow)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	dvs_msgs
	std_msgs
	sensor_msgs
	image_transport
	cv_bridge
	)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(

	CATKIN_DEPENDS
		roscpp
		image_transport
		cv_bridge
		dvs_msgs
		sensor_msgs
	DEPENDS
		OpenCV
		boost
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	)

# add_executable(talker src/talker.cpp)
# target_link_libraries(talker ${catkin_LIBRARIES})
# add_dependencies(talker ${catkin_EXPORTED_TARGETS})


# add_executable(listener src/listener.cpp)
# target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(listener  ${catkin_EXPORTED_TARGETS})


add_executable(event_viewer_node src/event_viewer_node.cpp src/event_viewer.cpp)
target_link_libraries(event_viewer_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(event_viewer_node ${catkin_EXPORTED_TARGETS})

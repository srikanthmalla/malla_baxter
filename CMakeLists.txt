cmake_minimum_required(VERSION 2.8.3)
project(malla_baxter)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	std_msgs
	moveit_commander
	roscpp
	rospy
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	pluginlib
	cmake_modules
	geometric_shapes
	visualization_msgs
)
set(Eigen_INCLUDE_DIRS "/usr/include/eigen3/Eigen/")
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)
find_package(Eigen REQUIRED)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(move_baxter src/move_baxter.cpp)
#target_link_libraries(move_baxter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(move_group src/moveit_interface.cpp)
#target_link_libraries(move_group ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(rviz_plotter src/rviz_plotter.cpp)
target_link_libraries(rviz_plotter ${catkin_LIBRARIES})
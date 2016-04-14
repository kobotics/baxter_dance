cmake_minimum_required(VERSION 2.8.3)
project(baxter_dance)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  baxter_core_msgs
  baxter_interface

  #added by Nikhil
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  cmake_modules
  geometric_shapes
)

#catkin_python_setup()

add_service_files(
  FILES
  ReactiveBehavior.srv
)

generate_messages(DEPENDENCIES std_msgs)

generate_dynamic_reconfigure_options(
  cfg_dance/JointSpringsExample.cfg
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  sensor_msgs
  control_msgs
  trajectory_msgs
  cv_bridge
  dynamic_reconfigure
  baxter_core_msgs
  baxter_interface

  #added by Nikhil
  moveit_core
  moveit_ros_planning_interface
  interactive_markers

)
#added by Nikhil
find_package(Eigen REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


add_executable(baxter_planning_scene_tutorial src/baxter_planning_scene_tutorial.cpp)
target_link_libraries(baxter_planning_scene_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS baxter_planning_scene_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(dance_collision_check src/dance_collision_check.cpp)
target_link_libraries(dance_collision_check ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS dance_collision_check DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(
#  DIRECTORY scripts/
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  USE_SOURCE_PERMISSIONS
#)

#install(
#  DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  USE_SOURCE_PERMISSIONS
#)

#install(
#  DIRECTORY share/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/share
#  USE_SOURCE_PERMISSIONS
#)

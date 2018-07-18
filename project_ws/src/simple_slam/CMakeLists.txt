cmake_minimum_required(VERSION 2.8.3)
project(simple_slam)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation rosgraph_msgs rospy std_msgs)

add_message_files(
  FILES
  landmark.msg
  lm_array.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rosgraph_msgs rospy std_msgs
)


install(PROGRAMS src/pyslam.py src/pyslam.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
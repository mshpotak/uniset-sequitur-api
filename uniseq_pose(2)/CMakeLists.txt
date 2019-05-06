cmake_minimum_required(VERSION 2.8.3)
project(uniseq_pose)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uniseq_pose
#  CATKIN_DEPENDS geometry_msgs sensor_msgs roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

add_executable( uniseq_pose src/uniseq_pose.cpp src/seq-req-class-v2.cpp)
target_link_libraries( uniseq_pose ${catkin_LIBRARIES} )

add_executable( uniseq_listen src/uniseq_listen.cpp )
target_link_libraries( uniseq_listen ${catkin_LIBRARIES} )

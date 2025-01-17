cmake_minimum_required(VERSION 3.0.2)
project(ir2425_group_24_a2)

## Compile as C++11, supported in ROS Kinetic and newer

find_package(catkin REQUIRED COMPONENTS
                roscpp
                actionlib
                actionlib_msgs
                move_base_msgs
                tiago_iaslab_simulation
				    std_msgs
				    message_generation
				    geometry_msgs
				    apriltag_ros
				    trajectory_msgs
                moveit_msgs
                moveit_ros_planning_interface
                moveit_core
                apriltag_ros
                tf2
                tf2_ros
                tf2_geometry_msgs
                control_msgs
                gazebo_ros_link_attacher
                gazebo_ros
)

find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   detection.msg
   picking_completed.msg
)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   manipulation.action
#   Action2.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   actionlib_msgs
   geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assignment2_ir2425_group24
    CATKIN_DEPENDS 
    roscpp 
    tiago_iaslab_simulation 
    message_runtime 
    geometry_msgs 
    trajectory_msgs 
    moveit_msgs
    moveit_ros_planning_interface
    moveit_core
    apriltag_ros
    tf2
    tf2_ros
    tf2_geometry_msgs
    control_msgs
    actionlib
    actionlib_msgs
    gazebo_ros_link_attacher
    gazebo_ros


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
 ${OpenCV_INCLUDE_DIRS}
)


add_executable(Node_A src/Node_A.cpp)
add_dependencies(Node_A ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Node_A ${catkin_LIBRARIES})

add_executable(Node_B src/Node_B.cpp)
add_dependencies(Node_B ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Node_B ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Node_C src/Node_C.cpp)
add_dependencies(Node_C ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Node_C ${catkin_LIBRARIES})

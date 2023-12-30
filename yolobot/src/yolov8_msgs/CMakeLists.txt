cmake_minimum_required(VERSION 3.0.2)
project(yolov8_msgs)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  geometry_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
    FILES
    InferenceResult.msg
    Yolov8Inference.msg
    DepthResult.msg
    PlotCoordinates.msg
    ConfidenceScore.msg
    CustomMsg.msg
)




# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yolov8_msgs
   CATKIN_DEPENDS std_msgs rospy
#  DEPENDS system_lib
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

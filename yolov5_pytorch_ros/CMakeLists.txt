cmake_minimum_required(VERSION 2.8.12)
project(yolov5_pytorch_ros)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    sensor_msgs
    std_msgs
    detection_msgs
    roscpp
    message_filters
    image_transport
    cv_bridge
    message_generation)

#add_message_files(
#  FILES
#    BoundingBox.msg
#    BoundingBoxes.msg
#)

#generate_messages(
#  DEPENDENCIES
#    geometry_msgs
#    sensor_msgs
#    std_msgs
#)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(detection_node src/cm_detect.cpp)
target_link_libraries(detection_node ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})



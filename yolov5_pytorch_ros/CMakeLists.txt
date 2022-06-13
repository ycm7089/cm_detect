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
    pcl_ros
    cv_bridge
    actionlib
    move_base_msgs
    tf
    tf_conversions
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

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_executable(detection_node src/cm_detect.cpp)
target_link_libraries(detection_node ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS} ${PCL_LIBRARIES} pcl_visualization)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_executable(Clustering_node src/cm_clustering.cpp)
target_link_libraries(Clustering_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} pcl_visualization)
add_definitions(${PCL_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS} )
add_executable(tf_node src/cm_tf.cpp)
target_link_libraries(tf_node ${catkin_LIBRARIES})



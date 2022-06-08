#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <yolov5_pytorch_ros/BoundingBox.h>
#include <yolov5_pytorch_ros/BoundingBoxes.h>
#include <detection_msgs/BoundingBoxes.h>

typedef struct
{
    char box_class;
    double box_probability;
    int box_xmin;
    int box_ymin;
    int box_xmax;
    int box_ymax;

} BoundingBox_Info;

typedef message_filters::sync_policies::ApproximateTime<detection_msgs::BoundingBoxes, sensor_msgs::Image> MySyncPolicy;

class cm_detect
{
public:
    yolov5_pytorch_ros::BoundingBox BoundingBox_msg;
    yolov5_pytorch_ros::BoundingBoxes BoundingBoxes_msg;

    ros::Subscriber realsense_RGB_sub;
    ros::Subscriber realsense_Depth_sub;
    ros::Subscriber BoundingBoxes_sub;

    BoundingBox_Info bounding_info;
    
    message_filters::Subscriber<detection_msgs::BoundingBoxes> image1_sub;
    message_filters::Subscriber<sensor_msgs::Image> image2_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;

    cm_detect(ros::NodeHandle *nh)
        : sync(MySyncPolicy(100), image1_sub, image2_sub)
    {
        std::cerr << "cm_detect()" << std::endl;

        image1_sub.subscribe(*nh, "/yolov5/detections", 100);
        image2_sub.subscribe(*nh, "/camera/depth/image_rect_raw", 100);

        sync.registerCallback(boost::bind(&cm_detect::realcallback, this,_1, _2));
        ROS_WARN_STREAM("z");
    }

public:
    void realsense_RGB_callback(const sensor_msgs::CameraInfo::Ptr&msg);
    void realsense_Depth_callback(const sensor_msgs::PointCloud2::Ptr&msg);
    void BoundingBoxes_callback(const yolov5_pytorch_ros::BoundingBoxes::ConstPtr& msg);
    void realcallback(const detection_msgs::BoundingBoxesConstPtr& bbox, const sensor_msgs::ImageConstPtr& depth);
};

void cm_detect::realcallback(const detection_msgs::BoundingBoxesConstPtr& bbox, const sensor_msgs::ImageConstPtr& depth)
{
    ROS_WARN_STREAM("T.T");
        ROS_WARN_STREAM("T.T");
            ROS_WARN_STREAM("T.T");

}

void cm_detect::realsense_RGB_callback(const sensor_msgs::CameraInfo::Ptr&msg)
{

}

void cm_detect::realsense_Depth_callback(const sensor_msgs::PointCloud2::Ptr&msg)
{

}

void cm_detect::BoundingBoxes_callback(const yolov5_pytorch_ros::BoundingBoxes::ConstPtr& msg)
{
    ROS_WARN_STREAM("BBox Callback");
    // bounding_info.box_xmax = msg -> xmax;
    // ROS_WARN_STREAM(bounding_info.box_xmax);
}

int main(int argc, char **argv) 
{
    // Main node
    ros::init(argc,argv,"detection_node");
    ros::NodeHandle nh("~");
    
    cm_detect data_sub(&nh);   

    // while loop
    ros::Rate loop_rate(1); //Hz
    while(ros::ok())
    {
        // data_sub.Tf_publisher();            
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

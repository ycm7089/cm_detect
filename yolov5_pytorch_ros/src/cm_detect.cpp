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

typedef struct
{
    char box_class;
    double box_probability;
    int box_xmin;
    int box_ymin;
    int box_xmax;
    int box_ymax;

} BoundingBox_Info;

class cm_detect
{
public:
    yolov5_pytorch_ros::BoundingBox BoundingBox_msg;
    yolov5_pytorch_ros::BoundingBoxes BoundingBoxes_msg;

    ros::Subscriber realsense_RGB_sub;
    ros::Subscriber realsense_Depth_sub;
    ros::Subscriber BoundingBoxes_sub;

    BoundingBox_Info bounding_info;
    
    cm_detect(ros::NodeHandle *nh)
    {
        std::cerr << "cm_detect()" << std::endl;
        //RGB Depth Boundingbox Sub
        // realsense_RGB_sub = nh->subscribe("/camera/color/camera_info",1, &cm_detect::realsense_RGB_callback,this);
        // realsense_Depth_sub = nh->subscribe("/camera/depth/image_raw",1, &cm_detect::realsense_Depth_callback,this); 
        // BoundingBoxes_sub = nh->subscribe("/yolov5/detections",1, &cm_detect::BoundingBoxes_callback,this); 

        // message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub (*nh,"/yolov5/detections",1);
        // message_filters::Subscriber<sensor_msgs::Image> depth_sub(*nh, "/camera/depth/image_raw", 1);
        // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::PointCloud2> sync(camera_info_sub, depth_sub, 10);
        // sync.registerCallback(boost::bind(&cm_detect::realcallback,this, _1, _2));

        message_filters::Subscriber<sensor_msgs::Image> image1_sub(*nh, "/camera/depth/image_rect_raw", 1);
        message_filters::Subscriber<sensor_msgs::CompressedImage> image2_sub(*nh, "/camera/color/image_raw/compressed", 1);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CompressedImage> MySyncPolicy;
        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
        sync.registerCallback(boost::bind(&cm_detect::realcallback, this,_1, _2));

    }
public:
    void realsense_RGB_callback(const sensor_msgs::CameraInfo::Ptr&msg);
    void realsense_Depth_callback(const sensor_msgs::PointCloud2::Ptr&msg);
    void BoundingBoxes_callback(const yolov5_pytorch_ros::BoundingBoxes::ConstPtr& msg);
    void realcallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CompressedImageConstPtr& cam_info);
};

void cm_detect::realcallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CompressedImageConstPtr& cam_info)
{
    ROS_WARN_STREAM("YES");
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

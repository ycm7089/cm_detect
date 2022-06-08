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

#include <detection_msgs/BoundingBox.h>
#include <detection_msgs/BoundingBoxes.h>

// below is fatal error: opencv2/core/core.hpp: No such file or directory
// #include <opencv2/core/core.hpp>

using namespace std;

typedef struct
{
    string box_class;
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
    // detection_msgs::BoundingBox BoundingBox_msg;
    // detection_msgs::BoundingBoxes BoundingBoxes_msg;

    BoundingBox_Info bounding_info;
    
    message_filters::Subscriber<detection_msgs::BoundingBoxes> bbox_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;

    cm_detect(ros::NodeHandle *nh)
        : sync(MySyncPolicy(100), bbox_sub, depth_sub)
    {
        std::cerr << "cm_detect()" << std::endl;

        bbox_sub.subscribe(*nh, "/yolov5/detections", 100);
        depth_sub.subscribe(*nh, "/camera/depth/image_rect_raw", 100);

        sync.registerCallback(boost::bind(&cm_detect::realcallback, this,_1, _2));
        ROS_WARN_STREAM("z");
    }

public:
   
    void realcallback(const detection_msgs::BoundingBoxesConstPtr& bbox, const sensor_msgs::ImageConstPtr& depth);
};

void cm_detect::realcallback(const detection_msgs::BoundingBoxesConstPtr& bbox, const sensor_msgs::ImageConstPtr& depth)
{
    ROS_WARN_STREAM("T.T");

    // Below code is a comparison bbox time and depth time because we want to know to use ApproximateTime
    // ROS_WARN_STREAM(bbox->header.stamp.sec);
    // std::cout << "BBox time : " << bbox -> header.stamp.sec << std::endl;
    // std::cout << "Depth Image time : " << depth -> header.stamp.sec << std::endl;

    // subscribe the bbox data
    bounding_info.box_class = bbox -> bounding_boxes[0].Class;
    bounding_info.box_probability = bbox -> bounding_boxes[0].probability;
    bounding_info.box_xmin = bbox -> bounding_boxes[0].xmin;
    bounding_info.box_ymin = bbox -> bounding_boxes[0].ymin;
    bounding_info.box_xmax = bbox -> bounding_boxes[0].xmax;
    bounding_info.box_ymax = bbox -> bounding_boxes[0].ymax;

    // https://cafepurple.tistory.com/46?category=1044213 depth info
    // std::cout << "Depth Info" << std::endl;
    // std::cout << "Size : " << depth.size() << std::endl;
    // std::cout << "Row, Col : " << depth.rows << depth.cols << std::endl;
    // std::cout << "Channels : " << depth.channels() << std::endl;
    // std::cout << "Element : " << depth.depth() << std::endl;
    // std::cout << "ElementSize : " << depth.elemSize() << std::endl;
    // std::cout << "Type : " << depth.type() << std::endl; // CV_8UC1 = 0

    // convert depth image in bbox to pointcloud
    if(bounding_info.box_class == "person" & bounding_info.box_probability >= 0.7)
    {
        ROS_WARN_STREAM("Yes");
        // cv::Mat convert_image;
        // convert_image.converTo();
    }
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

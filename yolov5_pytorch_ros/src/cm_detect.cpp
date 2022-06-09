#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <detection_msgs/BoundingBox.h>
#include <detection_msgs/BoundingBoxes.h>

// below is fatal error: opencv2/core/core.hpp: No such file or directory
// #include <opencv2/core/core.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

typedef struct
{
    std::string box_class;
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

    BoundingBox_Info bounding_info;
    
    message_filters::Subscriber<detection_msgs::BoundingBoxes> bbox_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;
    message_filters::Synchronizer<MySyncPolicy> sync;

    cm_detect(ros::NodeHandle *nh)
        : sync(MySyncPolicy(100), bbox_sub, depth_sub)
    {
        bbox_sub.subscribe(*nh, "/yolov5/detections", 100);
        depth_sub.subscribe(*nh, "/camera/depth/image_rect_raw", 100);

        sync.registerCallback(boost::bind(&cm_detect::realcallback, this,_1, _2));
    }

public:
   double f_x = 469.45703125;
   double f_y = 468.9375;
   double c_x = 353.50390625;
   double c_y = 243.98046875;


    void realcallback(const detection_msgs::BoundingBoxesConstPtr& bbox, const sensor_msgs::ImageConstPtr& depth);
};

void cm_detect::realcallback(const detection_msgs::BoundingBoxesConstPtr& bbox, const sensor_msgs::ImageConstPtr& depth)
{
    // subscribe the bbox data
    ROS_WARN_STREAM("===========");
    int size = bbox -> bounding_boxes.size();
    if (size > 0)
    {
        bounding_info.box_class = bbox -> bounding_boxes[0].Class;
        bounding_info.box_probability = bbox -> bounding_boxes[0].probability;
        bounding_info.box_xmin = bbox -> bounding_boxes[0].xmin;
        bounding_info.box_ymin = bbox -> bounding_boxes[0].ymin;
        bounding_info.box_xmax = bbox -> bounding_boxes[0].xmax;
        bounding_info.box_ymax = bbox -> bounding_boxes[0].ymax;

        // if (bounding_info.box_xmin < 0) bounding_info.box_xmin = 0;
        // if (bounding_info.box_xmax > 1279) bounding_info.box_xmax = 1279;
        // if (bounding_info.box_ymin < 0) bounding_info.box_ymin = 0;
        // if (bounding_info.box_ymax > 719) bounding_info.box_ymax = 719;

        ROS_INFO("x min : %d" ,bounding_info.box_xmin);
        ROS_INFO("x max : %d", bounding_info.box_xmax);

        ROS_INFO("y min : %d", bounding_info.box_ymin);
        ROS_INFO("y max : %d", bounding_info.box_ymax);
    
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::TYPE_16UC1); 
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception : %s", e.what());
            return;
        }

        cv::Mat& depth_mat = cv_ptr->image;
        // std::cout << "zz : " << depth_mat.size() << std::endl;

        double avr_v = (bounding_info.box_ymin + bounding_info.box_ymax)/2;
        double avr_u = (bounding_info.box_xmin + bounding_info.box_xmax)/2;

        double depth_x = 0.5 * bounding_info.box_xmax;
        double depth_y = 2 * bounding_info.box_ymax / 3;

        for(int u = bounding_info.box_xmin; u < bounding_info.box_xmax; u++)
        {
            for(int v = bounding_info.box_ymin; v < bounding_info.box_ymax; v++)    
            {
                double z = depth_mat.at<unsigned short>(avr_v ,avr_u) * 0.001; // unit : [m]
                // float z = depth_mat.at<unsigned short>(v ,u) * 0.001;
                if (z!=0) 
                {
                    // std::cout << "depth_unsigned_short : " << z << "m"<< std::endl;

                    double x = (u - c_x) * z / f_x;
                    double y = (v - c_y) * z / f_y;

                    std::cout << "x : " << x << " y : " << y << std::endl;
                }
            }
        }
    }

    // convert depth image in bbox to pointcloud
    if(bounding_info.box_class == "person" & bounding_info.box_probability >= 0.5)
    {
        // ROS_WARN_STREAM("Yes");
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
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

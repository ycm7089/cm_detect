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
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

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
    //L515
    // double f_x = 469.45703125;
    // double f_y = 468.9375;
    // double c_x = 353.50390625;
    // double c_y = 243.98046875;

    double f_x = 613.8178100585938;
    double f_y = 612.3670043945312;
    double c_x = 326.2353210449219;
    double c_y = 244.34783935546875;


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

        // ROS_INFO("x min : %d" ,bounding_info.box_xmin);
        // ROS_INFO("x max : %d", bounding_info.box_xmax);

        // ROS_INFO("y min : %d", bounding_info.box_ymin);
        // ROS_INFO("y max : %d", bounding_info.box_ymax);
    
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
        std::cout << "Size : " << depth_mat.size() << std::endl;
        ROS_WARN_STREAM("zzz");

        double avr_v = (bounding_info.box_ymin + bounding_info.box_ymax)/2;
        double avr_u = (bounding_info.box_xmin + bounding_info.box_xmax)/2;

        // int xmax_s = static_cast<int>(0.5 * bounding_info.box_xmax);
        // int xmin_s = static_cast<int>(0.5 * bounding_info.box_xmin);
        // int ymax_s = static_cast<int>(2.0 * bounding_info.box_ymax / 3.0);
        // int ymin_s = static_cast<int>(2.0 * bounding_info.box_ymin / 3.0);

        // int xmax_s = ( 848 * bounding_info.box_xmax ) / 640;        
        // int xmin_s = ( 848 * bounding_info.box_xmin ) / 640;

        int xmax_s = bounding_info.box_xmax;        
        int xmin_s = bounding_info.box_xmin;
        int ymax_s = bounding_info.box_ymax;
        int ymin_s = bounding_info.box_ymin;
        // To draw rect
        cv::Mat depth8;
        depth_mat.convertTo(depth8, CV_8UC1);
        cv::Mat depth8_3c;
        cv::cvtColor(depth8, depth8_3c, cv::COLOR_GRAY2RGB);

        cv::Rect rect(xmin_s, ymin_s, xmax_s-xmin_s, ymax_s-ymin_s);
        cv::rectangle(depth8_3c, rect, cv::Scalar(0,0,255), 2);

        cv::imshow("depth8_3c", depth8_3c);
        cv::waitKey(1);


        for(int u = xmin_s; u < xmax_s; u++)
        {
            for(int v = ymin_s; v < ymax_s; v++)
            {
                // cout << "(u, v) = " << "(" << u << ", " << v << ")" << endl;
                double z = depth_mat.at<unsigned short>(v ,u) * 0.001; // unit : [m]
                // float z = depth_mat.at<unsigned short>(v ,u) * 0.001;

                if (z!=0) 
                {
                    // std::cout << "depth_unsigned_short : " << z << "m"<< std::endl;

                    double x = (u - c_x) * z / f_x;
                    double y = (v - c_y) * z / f_y;

                    // cout << "x : " << x << " y : " << y << endl;
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

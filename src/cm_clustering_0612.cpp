#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <typeinfo>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include<pcl_conversions/pcl_conversions.h>

class cm_clustering
{
public:
    ros::Subscriber point_sub;
    ros::Publisher seg_pub;
    geometry_msgs::Point detection_point;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    cm_clustering(ros::NodeHandle *nh)
    {
      cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
      seg_pub = nh->advertise<sensor_msgs::PointCloud2>("PointXYZRGB",1);
      point_sub = nh->subscribe("/detection_node/PointXYZ",1, &cm_clustering::detection_callback,this); 
    }
    void detection_callback(const geometry_msgs::Point::Ptr&msg);
    void segmentation();
};

void cm_clustering::detection_callback(const geometry_msgs::Point::Ptr&msg)
{
  // ROS_WARN_STREAM("Yes");
  detection_point.x = msg -> x;
  detection_point.y = msg -> y;
  detection_point.z = msg -> z;
  // ROS_WARN_STREAM(detection_point.x);
  // ROS_WARN_STREAM(detection_point.y);
  // ROS_WARN_STREAM(detection_point.z);

  segmentation();
}

void cm_clustering::segmentation()
{
  // cloud = detection_point;
  // // *cloud.push_back(detection_point.x, detection_point.y, detection_point.z);

  // pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  // pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  // normal_estimator.setSearchMethod (tree);
  // normal_estimator.setInputCloud (cloud);
  // normal_estimator.setKSearch (50);
  // normal_estimator.compute (*normals);

  // pcl::IndicesPtr indices (new std::vector <int>);
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.0, 1.0);
  // pass.filter (*indices);

  // pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  // // reg.setMinClusterSize (100);
  // // reg.setMaxClusterSize (1000000);
  // // reg.setSearchMethod (tree);
  // // reg.setNumberOfNeighbours (30);
  // // reg.setInputCloud (cloud);
  // // // reg.setIndices (indices);
  // // reg.setInputNormals (normals);
  // // reg.setSmoothnessThreshold (4.0 / 180.0 * M_PI); //??? ????????? ????????? ????????? 1.0 ?????? ?????? ?????????~!!
  // // reg.setCurvatureThreshold (1.0);

  // reg.setMinClusterSize (50);
  // reg.setMaxClusterSize (1000000);
  // reg.setSearchMethod (tree);
  // reg.setNumberOfNeighbours (30);
  // reg.setInputCloud (cloud);
  // //reg.setIndices (indices);
  // reg.setInputNormals (normals);
  // reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  // reg.setCurvatureThreshold (1.0);

  // std::vector <pcl::PointIndices> clusters;
  // reg.extract (clusters);

  // std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  // std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  // std::cout << "These are the indices of the points of the initial" <<
  // std::endl << "cloud that belong to the first cluster:" << std::endl;
  // int counter = 0;
  // // while (counter < clusters[0].indices.size ())
  // // {
  //   // std::cout << clusters[0].indices[counter] << ", ";
  //   // counter++;
  //   // if (counter % 10 == 0)
  //     // std::cout << std::endl;
  // // }
  // // std::cout << std::endl;
  // sensor_msgs::PointCloud2 cloud_out;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  // std::cout << cloud->points[0].x << std::endl;
  // pcl::toROSMsg(*colored_cloud, cloud_out); //pcl -> pointcloud
  // ROS_WARN_STREAM("sadasd");
  // // cloud_out.header.frame_id = "camera_link";
  // // cloud_out.header.stamp = ros::Time::now();
  // ROS_WARN_STREAM("camera_link!");
  // seg_pub.publish(cloud_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Clustering_node");
  
  ros::NodeHandle nh("~");

  cm_clustering clustering(&nh);
  while (ros::ok())
  {
    ros::spinOnce();

  }  
  return (0);
}
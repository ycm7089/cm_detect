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

#include<pcl_conversions/pcl_conversions.h>

class cm_clustering
{
public:
    ros::Subscriber point_sub;
    ros::Publisher seg_pub;
    sensor_msgs::PointCloud2 detection_point;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    cm_clustering(ros::NodeHandle *nh)
    {
      cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
      seg_pub = nh->advertise<sensor_msgs::PointCloud2>("PointXYZRGB",1);
      point_sub = nh->subscribe("/detection_node/PointXYZ",1, &cm_clustering::detection_callback,this); 
    }
    void detection_callback(const sensor_msgs::PointCloud2::Ptr&msg);
    void segmentation();
};

void cm_clustering::detection_callback(const sensor_msgs::PointCloud2::Ptr&msg)
{
  ROS_WARN_STREAM("Yes");
  pcl::fromROSMsg(*msg,*cloud);

  segmentation();
}

void cm_clustering::segmentation()
{
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  // reg.setMinClusterSize (100);
  // reg.setMaxClusterSize (1000000);
  // reg.setSearchMethod (tree);
  // reg.setNumberOfNeighbours (30);
  // reg.setInputCloud (cloud);
  // // reg.setIndices (indices);
  // reg.setInputNormals (normals);
  // reg.setSmoothnessThreshold (4.0 / 180.0 * M_PI); //색 골고루 나오게 하려면 1.0 에서 증가 시키자~!!
  // reg.setCurvatureThreshold (1.0);

  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;

  sensor_msgs::PointCloud2 cloud_out;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  std::cout << cloud->points[0].x << std::endl;
  pcl::toROSMsg(*colored_cloud, cloud_out); //pcl -> pointcloud
  cloud_out.header.frame_id = "map";
  cloud_out.header.stamp = ros::Time::now();
  ROS_WARN_STREAM("map!");
  seg_pub.publish(cloud_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Clustering_node");
  
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(10); //Hz

  cm_clustering clustering(&nh);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return (0);
}
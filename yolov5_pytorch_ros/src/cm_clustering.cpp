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

    pcl::PointCloud<pcl::PointXYZ> cloud_dst;

    
    cm_clustering(ros::NodeHandle *nh)
    {
      cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
      seg_pub = nh->advertise<sensor_msgs::PointCloud2>("PointXYZRGB",1);
      point_sub = nh->subscribe("/detection_node/PointCloud",1, &cm_clustering::detection_callback,this); 
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
  // cloud point 의 normal_estimator(법선을 추정, 클라우드 포인트의 수직인 직선? )하여 normals 변수에 저장한다.

  //그분된 Point의 index 저장할 변수 설정
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc;
  filtered_pc.reset(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  //Z 축을 기준으로 0~1.0 meter 외의 point 값을 필터링 하기위해 pcl::PassThrough 함수 사용 
  pass.setFilterFieldName ("y"); // z축으로 filtering
  pass.setFilterLimits (0.4, 100.0); // 0.0 ~ 1.0 부분 추출 ??
  pass.setFilterLimitsNegative(true); //0.
  // pass.filter (*indices);
  pass.filter (*filtered_pc);
  // 바닥을 날려야하는데 못날리는중;
  ///////////////////////////////////////////////////////////////////
  // ROS_WARN_STREAM("ss");
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  // ROS_WARN_STREAM("ss222");
  normal_estimator.setInputCloud (filtered_pc);
  // ROS_WARN_STREAM("ss333");
  //setKSearch : target point cloud 안에서 고려할 가장 가까운 점들의 수
  normal_estimator.setKSearch (10);
  normal_estimator.compute (*normals);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);

  //KNN(레이블(정답)이 없는 예시를 분류하기 위한 알고리즘)을 찾는데 사용될 검색 방법을 설정  
  reg.setSearchMethod (tree);
  
  reg.setNumberOfNeighbours (30);
  // reg.setInputCloud (cloud);
  reg.setInputCloud (filtered_pc);

  // reg.setIndices (indices);
  reg.setInputNormals (normals);

  //각 point를 test 하기위한 smoothness thresold 를 설정 
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI); //색 골고루 나오게 하려면 1.0 에서 증가 시키자~!!
  
  //각 point를 test 하는데 사용되는 곡률의 임계값
  reg.setCurvatureThreshold (1.0);

  //분류된 각 point들의 index가 저장될 clusters 의 백터 변수 설정하고 뽑아냄.  
  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters); //군집화 적용
  // ROS_WARN_STREAM(clusters.size());
  if(clusters.size() > 0)
  {
    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl; //몇개의 segment로 분할 되었는지 로그로 확인
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
    std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;

    sensor_msgs::PointCloud2 cloud_out;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::toROSMsg(*colored_cloud, cloud_out); //pcl -> pointcloud
    // pcl::toROSMsg(*filtered_pc, cloud_out); //pcl -> pointcloud

    cloud_out.header.frame_id = "map";
    cloud_out.header.stamp = ros::Time::now();
    seg_pub.publish(cloud_out);

  }

  // ros::Duration(1.0).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Clustering_node");
  
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(30); //Hz

  cm_clustering clustering(&nh);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return (0);
}
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
#include <pcl/segmentation/extract_clusters.h>
#include <typeinfo>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include<pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>

class cm_clustering
{
public:
    int counter = 0;
    ros::Subscriber point_sub;
    ros::Publisher seg_pub;
    ros::Publisher geo_pub;
    
    tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Transform listener_tranform;
	tf::StampedTransform stamped_listener_tranform;
    tf::TransformListener listener;
    
    Eigen::Isometry3d eigen_transform;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;

    sensor_msgs::PointCloud2 cloud_out;
    geometry_msgs::PoseStamped BBox_pose;

    cm_clustering(ros::NodeHandle *nh)
    {
      cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
      tree.reset(new pcl::search::KdTree<pcl::PointXYZ>());

      seg_pub = nh->advertise<sensor_msgs::PointCloud2>("PointXYZRGB",1);
      geo_pub = nh -> advertise<geometry_msgs::PoseStamped> ("Center_PointXY", 1);
      point_sub = nh->subscribe("/detection_node/PointCloud",1, &cm_clustering::detection_callback,this); 
    }
    void detection_callback(const sensor_msgs::PointCloud2::Ptr& msg);
    void segmentation();
    void tf_publisher();
    void tf_listener();
    void region_growing_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                     pcl::PointCloud <pcl::Normal>::Ptr normals,
                                     std::vector <pcl::PointIndices>& indices);

    void euclidean_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                pcl::PointCloud <pcl::Normal>::Ptr normals,
                                std::vector <pcl::PointIndices>& indices);
};

void cm_clustering::detection_callback(const sensor_msgs::PointCloud2::Ptr& msg)
{
  //ROS_WARN_STREAM("Yes");
  if(counter % 15 == 0)
  {
      pcl::fromROSMsg(*msg,*cloud);
      segmentation();
  }
  
  counter++;
}

void cm_clustering::tf_listener()
{
    try
    {
        // bagfile ????????? ros::Time(0)
        // ????????? ros::Time::now()
        listener.lookupTransform("/map", "/camera_link",ros::Time(0), stamped_listener_tranform);
        listener_tranform = stamped_listener_tranform;

        tf::transformTFToEigen(listener_tranform, eigen_transform);
        // ROS_INFO("xx %.3f yy %.3f zz %.3f", transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
        
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        // continue;
    }
}

void cm_clustering::segmentation()
{
  

  //????????? Point??? index ????????? ?????? ??????
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc;
  filtered_pc.reset(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  //Z ?????? ???????????? 0~1.0 meter ?????? point ?????? ????????? ???????????? pcl::PassThrough ?????? ?????? 
  pass.setFilterFieldName ("y"); // z????????? filtering
  pass.setFilterLimits (0.4, 100.0); // 0.0 ~ 1.0 ?????? ?????? ??
  pass.setFilterLimitsNegative(true); //0.
  // pass.filter (*indices);
  pass.filter (*filtered_pc);

  // cloud point ??? normal_estimator(????????? ??????, ???????????? ???????????? ????????? ??????? )?????? normals ????????? ????????????.
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (filtered_pc);
  //setKSearch : target point cloud ????????? ????????? ?????? ????????? ????????? ???
  normal_estimator.setKSearch (10);
  normal_estimator.compute (*normals);

  std::vector <pcl::PointIndices> cluster_indices;

  //region_growing_segmentation(filtered_pc, normals, cluster_indices);
  euclidean_segmentation(filtered_pc, normals, cluster_indices);

  // ROS_WARN_STREAM(clusters.size());
  if(cluster_indices.size() > 0)
  {
    // std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl; //????????? segment??? ?????? ???????????? ????????? ??????
    // std::cout << "First cluster has " << cluster_indices[0].indices.size () << " points." << std::endl;
    // std::cout << "These are the indices of the points of the initial" <<
    // std::endl << "cloud that belong to the first cluster:" << std::endl;

    //https://adioshun.gitbooks.io/3d_people_detection/content/ebook/part02/part02-chapter01/part02-chapter01-euclidean.html
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // euclidean
    double max_num = -1;
    int max_idx = -1;

    for (int i=0; i < cluster_indices.size(); ++i)
    {
        std::cout << max_num << " < " << cluster_indices[i].indices.size() << std::endl;
        if(max_num < cluster_indices[i].indices.size())
        {
            max_num = (cluster_indices[i].indices.size());
            max_idx = i;
            std::cout << "max_idx is :" << max_idx << " max_num is : " << max_num << std::endl;
        }
    }

    if(max_idx == -1)
      return;
    else
    {
      if(max_idx == 1) ROS_WARN_STREAM("YAHO");
      int cm_count = 0;
      for(std::vector<int>::const_iterator pit = cluster_indices[max_idx].indices.begin (); 
          pit != cluster_indices[max_idx].indices.end (); ++pit)
      {
        //ROS_WARN_STREAM("222222");
        pcl::PointXYZRGB rgb_point;
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_point (new pcl::PointCloud<pcl::PointXYZRGB>);
        rgb_point.x = filtered_pc -> points[*pit].x;
        rgb_point.y = filtered_pc -> points[*pit].y;
        rgb_point.z = filtered_pc -> points[*pit].z;
        rgb_point.r = 255.0;
        rgb_point.g = 255.0;
        rgb_point.b = 0.0;
        rgb_point.a = 0.5;
        //a :: ?????? ?????? ?????? ????????? ?????? ????????? 0.0?????? ???????????? ?????? ?????? 1.0 ????????? ?????? ????????????.
        cloud_cluster->points.push_back (rgb_point); 
        // ROS_WARN_STREAM(filtered_pc -> points[0].x);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cm_count = cm_count + 1;
        // ROS_WARN_STREAM(cm_count);
        if( pit == cluster_indices[max_idx].indices.end () -1)
        {
          ROS_WARN_STREAM(cm_count);
          // ????????? ??? ????????????
          // ROS_WARN_STREAM(cluster_indices[max_idx].indices.end ());
          BBox_pose.pose.position.x = ( filtered_pc -> points[*cluster_indices[max_idx].indices.begin()].x + filtered_pc -> points[*pit].x ) /2;
          BBox_pose.pose.position.y = ( filtered_pc -> points[*cluster_indices[max_idx].indices.begin()].y + filtered_pc -> points[*pit].y ) /2;
          BBox_pose.pose.position.z = ( filtered_pc -> points[*cluster_indices[max_idx].indices.begin()].z + filtered_pc -> points[*pit].z ) /2;
         // ROS_WARN_STREAM(filtered_pc -> points[0].x);
         // ROS_WARN_STREAM(filtered_pc -> points[*pit].x);
         // ROS_WARN_STREAM(BBox_pose.position.x);
          
          tf_listener();
          Eigen::Vector3d pos;
          pos << BBox_pose.pose.position.x, BBox_pose.pose.position.y, BBox_pose.pose.position.z-0.5;
          
          Eigen::Vector3d transformed_pose = eigen_transform * pos;
          
          std::cout << pos.matrix() << std::endl;
          std::cout << "==" << std::endl;
          std::cout << transformed_pose.matrix() << std::endl;
          
          BBox_pose.header.stamp = ros::Time::now();
          BBox_pose.header.frame_id = "map"; 
          BBox_pose.pose.position.x = transformed_pose.x();
          BBox_pose.pose.position.y = transformed_pose.y();
          BBox_pose.pose.position.z = transformed_pose.z();          

          geo_pub.publish(BBox_pose);
        }

      }
      //map ??? ?????????????????? camera_link?????? tf listerner ?????????
      
      pcl::toROSMsg(*cloud_cluster, cloud_out); //pcl -> pointcloud
      cloud_out.header.frame_id = "camera_link";
      cloud_out.header.stamp = ros::Time::now();
      seg_pub.publish(cloud_out);
    }  
  
    // region_growing
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud (); 
    //pcl::toROSMsg(*colored_cloud, cloud_out); //pcl -> pointcloud
    
    //cloud_out.header.frame_id = "camera_link";
    //cloud_out.header.stamp = ros::Time::now();
    //seg_pub.publish(cloud_out);

  }
  // ros::Duration(1.0).sleep();
}

void cm_clustering::region_growing_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                pcl::PointCloud <pcl::Normal>::Ptr normals,
                                                std::vector <pcl::PointIndices>& indices)
{
  
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);

  //KNN(?????????(??????)??? ?????? ????????? ???????????? ?????? ????????????)??? ????????? ????????? ?????? ????????? ??????  
  reg.setSearchMethod (tree);
  
  reg.setNumberOfNeighbours (30);
  // reg.setInputCloud (cloud);
  reg.setInputCloud (cloud);

  // reg.setIndices (indices);
  reg.setInputNormals (normals);

  //??? point??? test ???????????? smoothness thresold ??? ?????? 
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI); //??? ????????? ????????? ????????? 1.0 ?????? ?????? ?????????~!!
  
  //??? point??? test ????????? ???????????? ????????? ?????????
  reg.setCurvatureThreshold (1.0);

  //????????? ??? point?????? index??? ????????? clusters ??? ?????? ?????? ???????????? ?????????.  
  
  reg.extract (indices); //????????? ??????
}

void cm_clustering::euclidean_segmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                           pcl::PointCloud <pcl::Normal>::Ptr normals,
                                           std::vector <pcl::PointIndices>& indices)
{
  
  ec.setInputCloud (cloud);       // ??????   
  ec.setClusterTolerance (0.05);  // 2cm  ??????????????????!!!
  ec.setMinClusterSize (30);     // ?????? ????????? ??? 
  ec.setMaxClusterSize (2500000);   // ?????? ????????? ???
  ec.setSearchMethod (tree);      // ????????? ????????? ?????? ?????? ?????? 
  ec.extract (indices);   // ????????? ??????
}

void cm_clustering::tf_publisher()
{
  transform.setOrigin( tf::Vector3(0.25, 0.0, 0.4) );
  transform.setRotation(tf::createQuaternionFromRPY(-90.0*M_PI/180.0, 0.0, -90.0*M_PI/180.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Clustering_node");
  
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(300); //Hz

  cm_clustering clustering(&nh);
  while (ros::ok())
  {
    clustering.tf_publisher();
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return (0);
}

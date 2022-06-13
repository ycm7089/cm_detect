#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdio>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

class cm_move_base
{
public:
  tf::TransformBroadcaster br;
	tf::Transform transform;
  ros::Subscriber BBox_pose_info_sub;
  ros::Subscriber amcl_info_sub;

  geometry_msgs::Pose BBox_pose;

  string s;
  double x;
  double y;
  
  cm_move_base(ros::NodeHandle *nh)
  {
    BBox_pose_info_sub = nh -> subscribe("/detection_node/PoseXYZ",1, &cm_move_base::sub_BBox_pose_MsgCallback, this);
  }
  void sub_BBox_pose_MsgCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void tf_publisher();
};

void cm_move_base::sub_BBox_pose_MsgCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  BBox_pose.position.x = msg -> position.x;
  BBox_pose.position.y = msg -> position.y;
  BBox_pose.position.z = msg -> position.z;

  transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, atan2(BBox_pose.position.x, BBox_pose.position.z)));

  BBox_pose.orientation.z = transform.getRotation().z();
  ros::Duration(1.0).sleep();

  ROS_WARN_STREAM("------------------------------");
  ROS_WARN_STREAM("rotation z radian is " << atan2(BBox_pose.position.x, BBox_pose.position.z)); // -pi ~ pi
  ROS_WARN_STREAM("position x is " << BBox_pose.position.x << "m");
  ROS_WARN_STREAM("position y is " << BBox_pose.position.y << "m");
  ROS_WARN_STREAM("position z is " << BBox_pose.position.z << "m");
  ROS_WARN_STREAM("Quaternion z is " << BBox_pose.orientation.z); // Yaw
  ROS_WARN_STREAM("==============================");
}

void cm_move_base::tf_publisher()
{

  transform.setOrigin( tf::Vector3(0.25, 0.0, 0.4) );
  transform.setRotation(tf::createQuaternionFromRPY(-90.0*M_PI/180.0, 0.0, -90.0*M_PI/180.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera"));
  // transform.setOrigin( tf::Vector3(BBox_pose.position.x, BBox_pose.position.y, BBox_pose.position.z) );
  // tf::Quaternion MarkerQ(BBox_pose.orientation.x,BBox_pose.orientation.y,BBox_pose.orientation.z,BBox_pose.orientation.w); // yaw pitch roll
  // transform.setRotation(MarkerQ);
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_0", "marker"));

  // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.6 ));
  // transform.setRotation(tf::createQuaternionFromRPY(0.0,90.0*M_PI/180,0.0));
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "marker", "safe_link"));

  transform.setOrigin( tf::Vector3(0.0, 0.0, -0.6) );
  // transform.setRotation(MarkerQ);
  transform.setRotation(tf::createQuaternionFromRPY(0.0,-90.0*M_PI/180,90.0*M_PI/180));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "safe_link"));

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh("~");

  cm_move_base cm_move_bases(&nh);  
 
  // ros::Rate loop_rate(10);
  while(ros::ok())
  {
    cm_move_bases.tf_publisher();
    ros::spinOnce();
    // loop_rate.sleep();
  }
  return 0;
}

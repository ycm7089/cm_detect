#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdio>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class cm_move_base
{
public:
  tf::TransformBroadcaster br;
	tf::Transform transform;
  ros::Subscriber Cluster_pose_info_sub;
  ros::Subscriber amcl_info_sub;

  move_base_msgs::MoveBaseGoal goal;

  geometry_msgs::PoseStamped center_point_pose;

  string s;
  double x;
  double y;
  
  cm_move_base(ros::NodeHandle *nh)
  {
    Cluster_pose_info_sub = nh -> subscribe("/Clustering_node/Center_PointXY",1, &cm_move_base::Cluster_pose_MsgCallback, this);
  }
  void Cluster_pose_MsgCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void Object_tracking();

};

void cm_move_base::Object_tracking()
{
  MoveBaseClient ac("move_base", true);
   while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    } 
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    // goal.target_pose.pose.position.x = transform.getOrigin().x();
    // goal.target_pose.pose.position.y = transform.getOrigin().y();    
    goal.target_pose.pose.position.x = center_point_pose.pose.position.x;
    goal.target_pose.pose.position.y = center_point_pose.pose.position.y;
    goal.target_pose.pose.position.z = center_point_pose.pose.position.z;

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;    
    goal.target_pose.pose.orientation.z = atan2(center_point_pose.pose.position.x, center_point_pose.pose.position.z);
    goal.target_pose.pose.orientation.w = 0.0;

    ac.sendGoal(goal);
}

void cm_move_base::Cluster_pose_MsgCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  center_point_pose.pose.position.x = msg -> pose.position.x;
  center_point_pose.pose.position.y = msg -> pose.position.y;
  center_point_pose.pose.position.z = msg -> pose.position.z;

  transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, atan2(center_point_pose.pose.position.x, center_point_pose.pose.position.z)));

  center_point_pose.pose.orientation.z = transform.getRotation().z();
  // ros::Duration(1.0).sleep();

  ROS_WARN_STREAM("------------------------------");
  ROS_WARN_STREAM("rotation z radian is " << atan2(center_point_pose.pose.position.x, center_point_pose.pose.position.z)); // -pi ~ pi
  ROS_WARN_STREAM("position x is " << center_point_pose.pose.position.x << "m");
  ROS_WARN_STREAM("position y is " << center_point_pose.pose.position.y << "m");
  ROS_WARN_STREAM("position z is " << center_point_pose.pose.position.z << "m");
  ROS_WARN_STREAM("Quaternion z is " << center_point_pose.pose.orientation.z); // Yaw
  ROS_WARN_STREAM("==============================");

  // Object_tracking();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cm_move_base_node");
  ros::NodeHandle nh("~");

  cm_move_base cm_move_bases(&nh);  
 
  // ros::Rate loop_rate(10);
  while(ros::ok())
  {
    ros::spinOnce();
    // loop_rate.sleep();
  }
  return 0;
}

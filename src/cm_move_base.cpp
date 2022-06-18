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
  ros::Publisher pose_pub;

  move_base_msgs::MoveBaseGoal goal;

  geometry_msgs::PoseStamped visual_orienta;
  geometry_msgs::PoseStamped center_point_pose;
  
  cm_move_base(ros::NodeHandle *nh)
  {
    Cluster_pose_info_sub = nh -> subscribe("/Clustering_node/Center_PointXY",1, &cm_move_base::Cluster_pose_MsgCallback, this);
    pose_pub = nh -> advertise<geometry_msgs::PoseStamped>("/arrow_visualization", 1);

  }
  void Cluster_pose_MsgCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void Object_tracking();
  void pub_arrow();
};

void cm_move_base::pub_arrow()
{
    // we need Yaw 
    geometry_msgs::Quaternion newQ(tf::createQuaternionMsgFromYaw(goal.target_pose.pose.position.z ));
    goal.target_pose.pose.orientation.x = newQ.x;
    goal.target_pose.pose.orientation.y = newQ.y;
    goal.target_pose.pose.orientation.z = newQ.z;
    goal.target_pose.pose.orientation.w = newQ.w;    

    visual_orienta.header.frame_id = "map";
    visual_orienta.header.stamp=ros::Time::now();
    visual_orienta.pose.position.x = goal.target_pose.pose.position.x;
    visual_orienta.pose.position.y = goal.target_pose.pose.position.y;
    visual_orienta.pose.position.z = goal.target_pose.pose.position.z;

    visual_orienta.pose.orientation.x =goal.target_pose.pose.orientation.x;
    visual_orienta.pose.orientation.y =goal.target_pose.pose.orientation.y;
    visual_orienta.pose.orientation.z =goal.target_pose.pose.orientation.z;
    visual_orienta.pose.orientation.w =goal.target_pose.pose.orientation.w;
    
    ROS_WARN_STREAM(visual_orienta);
    pose_pub.publish(visual_orienta);
}

void cm_move_base::Object_tracking()
{
  MoveBaseClient ac("move_base", true);
  //  while(!ac.waitForServer(ros::Duration(5.0)))
  //   {
  //       ROS_INFO("Waiting for the move_base action server to come up");
  //   }
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
    ROS_WARN_STREAM(goal);

    ac.sendGoal(goal);
    // pub_arrow();
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
  ros::init(argc, argv, "cm_move_base");
  ros::NodeHandle nh("~");

  cm_move_base cm_move_bases(&nh);  
 
  // ros::Rate loop_rate(10);
  while(ros::ok())
  {
    cm_move_bases.Object_tracking();
    cm_move_bases.pub_arrow();
    ros::spinOnce();
    // loop_rate.sleep();
  }
  return 0;
}

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

  // tf::Transform transform2;
  tf::TransformListener listener;
  tf::StampedTransform transform2;

  ros::Subscriber Cluster_pose_info_sub;
  ros::Publisher pose_pub;

  MoveBaseClient ac;

  move_base_msgs::MoveBaseGoal goal;

  geometry_msgs::PoseStamped visual_orienta;
  geometry_msgs::PoseStamped center_point_pose;

  geometry_msgs::PoseStamped previous_point_pose;

  int callback_count = 0;  
  double Yaw = 0.0;
  double r = 0.0;
  double p = 0.0;
  double y = 0.0;
  cm_move_base(ros::NodeHandle *nh)
    :ac("move_base", true)
  {
    Cluster_pose_info_sub = nh -> subscribe("/Clustering_node/Center_PointXY",1, &cm_move_base::Cluster_pose_MsgCallback, this);
    pose_pub = nh -> advertise<geometry_msgs::PoseStamped>("/arrow_visualization", 1);
  }
  void Cluster_pose_MsgCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void Object_tracking();
  void pub_arrow();
  void tf_listener();
  void tf_publisher();
};

void cm_move_base::pub_arrow()
{
    // we need Yaw 
    // geometry_msgs::Quaternion newQ(tf::createQuaternionMsgFromYaw(goal.target_pose.pose.position.z ));
    geometry_msgs::Quaternion newQ(tf::createQuaternionMsgFromYaw(Yaw ));
    // geometry_msgs::Quaternion newQ(tf::createQuaternionMsgFromYaw(y ));
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
    
    // ROS_WARN_STREAM(visual_orienta);
    pose_pub.publish(visual_orienta);
    ROS_WARN_STREAM( "Yaw is " << Yaw <<" count is " << callback_count);

}

void cm_move_base::Object_tracking()
{
  //  while(!ac.waitForServer(ros::Duration(5.0)))
  //   {
  //       ROS_INFO("Waiting for the move_base action server to come up");
  //   }
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
 
    goal.target_pose.pose.position.x = center_point_pose.pose.position.x;
    goal.target_pose.pose.position.y = center_point_pose.pose.position.y;
    goal.target_pose.pose.position.z = center_point_pose.pose.position.z;
    // goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;    
    goal.target_pose.pose.orientation.z = center_point_pose.pose.orientation.z; // need Quaternion 
    goal.target_pose.pose.orientation.w = 0.0;

    // goal.target_pose.pose.position.x = transform2.getOrigin().x();
    // goal.target_pose.pose.position.y = transform2.getOrigin().y();  

    ROS_WARN_STREAM(goal);
    ros::Duration(1.5).sleep();
    ac.sendGoal(goal);
    pub_arrow();
}

void cm_move_base::tf_publisher()
{
  transform.setOrigin( tf::Vector3(center_point_pose.pose.position.x, center_point_pose.pose.position.y, center_point_pose.pose.position.z) );
  transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "safe_link"));
}

void cm_move_base::tf_listener()
{
  try
  {
    listener.lookupTransform("/base_link", "/safe_link", ros::Time(0), transform2);
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

void cm_move_base::Cluster_pose_MsgCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  
  center_point_pose.pose.position.x = msg -> pose.position.x;
  center_point_pose.pose.position.y = msg -> pose.position.y;
  center_point_pose.pose.position.z = msg -> pose.position.z;

  // transform.setOrigin(tf::Vector3(transform2.getOrigin().x(),transform2.getOrigin().y(),transform2.getOrigin().z()));
  // tf::Quaternion MarkerQ(transform2.getRotation().x(),transform2.getRotation().y(),transform2.getRotation().z());

  // tf:: Matrix3x3 m(MarkerQ);
  // m.getRPY(r,p,y);


  if (callback_count == 0){
    // transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0.0));
    center_point_pose.pose.orientation.z = 0.0;
    Yaw = 0.0;

    previous_point_pose.pose.position.x = center_point_pose.pose.position.x;
    previous_point_pose.pose.position.y = center_point_pose.pose.position.y;
    previous_point_pose.pose.position.z = center_point_pose.pose.position.z;

    callback_count = callback_count + 1;
  } 

  else{
    // transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, atan2(center_point_pose.pose.position.x - previous_point_pose.pose.position.x, center_point_pose.pose.position.z - previous_point_pose.pose.position.z)));
    // Yaw = atan2(center_point_pose.pose.position.x - previous_point_pose.pose.position.x, center_point_pose.pose.position.z - previous_point_pose.pose.position.z);
    
    if(((center_point_pose.pose.position.x - previous_point_pose.pose.position.x) * (center_point_pose.pose.position.x - previous_point_pose.pose.position.x )
     + (center_point_pose.pose.position.y - previous_point_pose.pose.position.y) * (center_point_pose.pose.position.y - previous_point_pose.pose.position.y ))
    > (0.3 * 0.3) )
    {
      transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 
      atan2(center_point_pose.pose.position.y - previous_point_pose.pose.position.y,center_point_pose.pose.position.x - previous_point_pose.pose.position.x)));
      Yaw = atan2( center_point_pose.pose.position.y - previous_point_pose.pose.position.y,center_point_pose.pose.position.x - previous_point_pose.pose.position.x);
      center_point_pose.pose.orientation.z = transform.getRotation().z();
      ROS_WARN_STREAM(Yaw);
  
  


      ROS_WARN_STREAM("------------------------------");
      ROS_WARN_STREAM("rotation z radian is " << atan2(center_point_pose.pose.position.x - previous_point_pose.pose.position.x, center_point_pose.pose.position.z - previous_point_pose.pose.position.z)); // -pi ~ pi
      ROS_WARN_STREAM("cur_position x is " << center_point_pose.pose.position.x << "m");
      ROS_WARN_STREAM("cur_position y is " << center_point_pose.pose.position.y << "m");
      ROS_WARN_STREAM("pre_position x is " << previous_point_pose.pose.position.x << "m");
      ROS_WARN_STREAM("pre_position y is " << previous_point_pose.pose.position.y << "m");
            ROS_WARN_STREAM("Quaternion z is " << center_point_pose.pose.orientation.z); // Yaw
      ROS_WARN_STREAM("==============================");
      
      previous_point_pose.pose.position.x = center_point_pose.pose.position.x;
      previous_point_pose.pose.position.y = center_point_pose.pose.position.y;
      previous_point_pose.pose.position.z = center_point_pose.pose.position.z;




      Object_tracking();

      callback_count = callback_count + 1;
    }

 
  }

  // ros::Duration(1.0).sleep();

  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cm_move_base");
  ros::NodeHandle nh("~");

  cm_move_base cm_move_bases(&nh);  
 
  ros::Rate loop_rate(2);
  while(ros::ok())
  {
    // cm_move_bases.Object_tracking();
    // cm_move_bases.pub_arrow();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

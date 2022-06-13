#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class cm_tf_listener
{
public:
    // TF
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    // Movebase
    move_base_msgs::MoveBaseGoal goal;

    // Arrow shape marker
    geometry_msgs::PoseStamped visual_orienta;

    ros::Publisher pose_pub;

    double r,p,y;

    cm_tf_listener(ros::NodeHandle *nh)
    {
        pose_pub = nh -> advertise<geometry_msgs::PoseStamped>("/marker_visualization", 1);
    }
    void tf_listener();
    void marker_tracking();
    void pub_Marker();
};

void cm_tf_listener::tf_listener()
{
    try
    {
        listener.lookupTransform("/map", "/safe_link",ros::Time(0), transform);
        // ROS_INFO("xx %.3f yy %.3f zz %.3f", transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
        
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        // continue;
    }
}

void cm_tf_listener::marker_tracking()
{
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    } 
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = transform.getOrigin().x();
    goal.target_pose.pose.position.y = transform.getOrigin().y();    
    
    // ROS_INFO("goal_pose  x is %.3f y is %.3f", goal.target_pose.pose.position.x ,goal.target_pose.pose.position.y );

    // version 1

    // double sqw = pow(transform.getRotation().w(),2);
    // double sqx = pow(transform.getRotation().x(),2);
    // double sqy = pow(transform.getRotation().y(),2);
    // double sqz= pow(transform.getRotation().z(),2);
    // double yaw = atan2f(2.0f *(transform.getRotation().z() * transform.getRotation().w() + 
    // transform.getRotation().x() * transform.getRotation().y()), sqw + sqx - sqy - sqz);

    //vesrion 2
    transform.setOrigin( tf::Vector3(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()) );
    tf::Quaternion MarkerQ(transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w()); // yaw pitch roll
    // transform.setRotation(MarkerQ);

    tf::Matrix3x3 m(MarkerQ);

    m.getRPY(r,p,y);

    // ROS_INFO("Sending goal");
    ROS_INFO("Goal Pose Yaw is %.3f", y);

    ac.sendGoal(goal);
    
    // ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("YES!");    

    //we should change from global path to local path
    //to do this we should fix the costamp
}

void cm_tf_listener::pub_Marker()
{
    // we need Yaw 
    geometry_msgs::Quaternion newQ(tf::createQuaternionMsgFromYaw(y));
        
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
    
    pose_pub.publish(visual_orienta);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");
    ros::NodeHandle nh("~");
    cm_tf_listener cm_tf_listeners(&nh);
    ros::Rate rate(2.0);

    while (ros::ok())
    {
        cm_tf_listeners.tf_listener();
        cm_tf_listeners.marker_tracking();
        cm_tf_listeners.pub_Marker();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
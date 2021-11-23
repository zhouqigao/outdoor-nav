#include <ros/ros.h> 
#include <ros/console.h> 
#include <nav_msgs/Path.h> 
#include <std_msgs/String.h> 
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h>
main (int argc, char **argv)
{ 
  ros::init (argc, argv, "showpath"); 
 
  ros::NodeHandle ph; 
  ros::Publisher path_pub1 = ph.advertise<nav_msgs::Path>("trajectory1",1, true); 
  ros::Publisher path_pub2 = ph.advertise<nav_msgs::Path>("trajectory2",1, true); 

 
  ros::Time current_time, last_time; 
  current_time = ros::Time::now(); 
  last_time = ros::Time::now(); 
 
  nav_msgs::Path path1; 
  nav_msgs::Path path2; 

  //nav_msgs::Path path1; 
  path1.header.stamp=current_time; 
  path1.header.frame_id="base_link"; 
 

  path2.header.stamp=current_time; 
  path2.header.frame_id="base_link";

  double x1 = 0.0; 
  double y1 = 0.0; 
  double x2 = 0.0; 
  double y2 = 0.0; 

  double th = 0.0; 
  double r1 = 30;
  double r2 = 90;
  double vx = 0.1; 
  double vy = -0.1; 
  double vth = 0.2; 
 
  ros::Rate loop_rate(1); 
while (ros::ok()) 
{ 
   current_time = ros::Time::now(); 
   //compute odometry in a typical way given the velocities of the robot 
   double dt = (current_time - last_time).toSec(); 
  //  double dt = 3;
  //  double delta_x = (vx * cos(th) - vy * sin(th)) * dt; 
  //  double delta_y = (vx * sin(th) + vy * cos(th)) * dt; 
	x1 = r1*sin(th);
	y1 = r1*cos(th);
  	x2 = r2*sin(th);
	y2 = r2*cos(th);
   double delta_th = vth * dt; 
 
  //  x += delta_x;
  //  y += delta_y; 
   th += delta_th; 
 
   geometry_msgs::PoseStamped this_pose_stamped1; 
   geometry_msgs::PoseStamped this_pose_stamped2; 

   this_pose_stamped1.pose.position.x = x1; 
   this_pose_stamped1.pose.position.y = y1;
      this_pose_stamped2.pose.position.x = x2; 
   this_pose_stamped2.pose.position.y = y2;
 
   geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th); 
   this_pose_stamped1.pose.orientation.x = goal_quat.x; 
   this_pose_stamped1.pose.orientation.y = goal_quat.y; 
   this_pose_stamped1.pose.orientation.z = goal_quat.z; 
   this_pose_stamped1.pose.orientation.w = goal_quat.w; 
 
   this_pose_stamped1.header.stamp=current_time; 
   this_pose_stamped1.header.frame_id="base_link"; 
   path1.poses.push_back(this_pose_stamped1); 


      this_pose_stamped2.pose.orientation.x = goal_quat.x; 
   this_pose_stamped2.pose.orientation.y = goal_quat.y; 
   this_pose_stamped2.pose.orientation.z = goal_quat.z; 
   this_pose_stamped2.pose.orientation.w = goal_quat.w; 
 
   this_pose_stamped2.header.stamp=current_time; 
   this_pose_stamped2.header.frame_id="base_link"; 
   path2.poses.push_back(this_pose_stamped2); 


   path_pub1.publish(path1);
   path_pub2.publish(path2);

   ros::spinOnce();               
   // check for incoming messages 
 
   last_time = current_time; 
   loop_rate.sleep(); 
  } 
return 0; 
}

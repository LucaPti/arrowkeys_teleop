#include "ros/ros.h"
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "twist2rc");
  ros::NodeHandle n_; 
  ros::Publisher pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ROS_INFO("publishing to cmd_vel");

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    
    geometry_msgs::Twist msg;
    msg.linear.x = 0.25;
    msg.angular.z = 0.1;
    pub_.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

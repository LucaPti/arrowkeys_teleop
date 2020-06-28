#include "ros/ros.h"
#include <geometry_msgs/Twist.h>


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>
#include <time.h>
#include <stdint.h>
// ioctl stuff runs only as root -> sudo -s before rosrun

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "twist2rc");
  ros::NodeHandle n_; 
  ros::Publisher pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ROS_INFO("publishing to cmd_vel");
  ros::Rate loop_rate(100);

  // Keyboard stuff
  int rcode = 0;

  char keyboard_name[256] = "Unknown";
  int keyboard_fd = open("/dev/input/event1", O_RDONLY | O_NONBLOCK);
  if ( keyboard_fd == -1 ) {
    printf("Failed to open keyboard.\n");
    exit(1);
  }
  rcode = ioctl(keyboard_fd, EVIOCGNAME(sizeof(keyboard_name)), keyboard_name);
  printf("Reading From : %s \n", keyboard_name);

  printf("Getting exclusive access: ");
  rcode = ioctl(keyboard_fd, EVIOCGRAB, 1);
  printf("%s\n", (rcode == 0) ? "SUCCESS" : "FAILURE");
  struct input_event keyboard_event;
int end = time(NULL) + 10;
// Goal: detect arrow key to get exclusive access, esc restores old mode
  while (ros::ok() && (time(NULL)<end))
  {
    if ( read(keyboard_fd, &keyboard_event, sizeof(keyboard_event)) != -1 ) {
      printf("keyboard event: type %d code %d value %d  \n", keyboard_event.type, keyboard_event.code, keyboard_event.value);
    }
    geometry_msgs::Twist msg;
    msg.linear.x = 0.25;
    msg.angular.z = 0.1;
    pub_.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
printf("Exiting.\n");
  rcode = ioctl(keyboard_fd, EVIOCGRAB, 1);
  close(keyboard_fd);
  return 0;
}

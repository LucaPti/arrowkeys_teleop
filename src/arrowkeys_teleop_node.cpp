#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#include <gainput/gainput.h>
#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <GL/glx.h>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <arrowkeys_teleop/ArrowKeysConfig.h>

enum Buttons
{
  ArrowLeft,
  ArrowRight,
  ArrowUp,
  ArrowDown
};

const char* windowName = "Arrow Key Input Node";
const int width = 400;
const int height = 400;
double forward_range = 0.25;
double reverse_range = 0.25;
double steering_range = 1;

void callback(arrowkeys_teleop::ArrowKeysConfig &config, uint32_t level) {
  forward_range = config.forward_range;
  reverse_range = config.reverse_range;
  steering_range = config.steering_range;
}

int main(int argc, char **argv)
{
  // Initiate ROS
  ros::init(argc, argv, "arrowkeys_teleop_node");
  ros::NodeHandle n_; 
  ros::Publisher pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ROS_INFO("publishing to cmd_vel");
  ros::Rate loop_rate(50);
  dynamic_reconfigure::Server<arrowkeys_teleop::ArrowKeysConfig> server;
  dynamic_reconfigure::Server<arrowkeys_teleop::ArrowKeysConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Initiate X Window Manager
  static int attributeListDbl[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_RED_SIZE, 1, GLX_GREEN_SIZE, 1, GLX_BLUE_SIZE, 1, None};

  Display* xDisplay = XOpenDisplay(0);
  if (xDisplay == 0)
  {
    std::cerr << "Cannot connect to X server." << std::endl;
    return -1;
  }

  Window root = DefaultRootWindow(xDisplay);

  XVisualInfo* vi = glXChooseVisual(xDisplay, DefaultScreen(xDisplay), attributeListDbl);
  assert(vi);

  GLXContext context = glXCreateContext(xDisplay, vi, 0, GL_TRUE);

  Colormap cmap = XCreateColormap(xDisplay, root, vi->visual, AllocNone);

  XSetWindowAttributes swa;
  swa.colormap = cmap;
  swa.event_mask = ExposureMask
		| KeyPressMask | KeyReleaseMask
		| PointerMotionMask | ButtonPressMask | ButtonReleaseMask;

  Window xWindow = XCreateWindow(
			xDisplay, root,
			0, 0, width, height, 0,
			CopyFromParent, InputOutput,
			CopyFromParent, CWEventMask,
			&swa
			);

  glXMakeCurrent(xDisplay, xWindow, context);

  XSetWindowAttributes xattr;
  xattr.override_redirect = False;
  XChangeWindowAttributes(xDisplay, xWindow, CWOverrideRedirect, &xattr);

  XMapWindow(xDisplay, xWindow);
  XStoreName(xDisplay, xWindow, windowName);

  // Setup Gainput
  gainput::InputManager manager;
  const gainput::DeviceId keyboardId = manager.CreateDevice<gainput::InputDeviceKeyboard>();

  gainput::InputMap map(manager);
  map.MapBool(ArrowLeft, keyboardId, gainput::KeyLeft);
  map.MapBool(ArrowRight, keyboardId, gainput::KeyRight);
  map.MapBool(ArrowUp, keyboardId, gainput::KeyUp);
  map.MapBool(ArrowDown, keyboardId, gainput::KeyDown);

  manager.SetDisplaySize(width, height);

  while (ros::ok())
  {
    // Update Gainput
    manager.Update();

    XEvent event;
    while (XPending(xDisplay))
    {
      XNextEvent(xDisplay, &event);
      manager.HandleEvent(event);
    }

    // Check button states
    geometry_msgs::Twist msg;
    if (map.GetBool(ArrowLeft))
    {
      msg.angular.z = steering_range;
    }
    else if (map.GetBool(ArrowRight))
    {
      msg.angular.z = -steering_range;
    }
    else 
    {
      msg.angular.z = 0.0;
    }
    if (map.GetBool(ArrowUp))
    {
      msg.linear.x = forward_range;
    }
    else if (map.GetBool(ArrowDown))
    {
      msg.linear.x = -reverse_range;
    }
    else 
    {
      msg.linear.x = 0.0;
    }
    pub_.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}

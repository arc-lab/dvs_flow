#include "dvs_flow/event_viewer.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

int main(int argc, char **argv)
{

  ROS_INFO("Event Listener Started");
  ros::init(argc, argv, "event_viewer");

  ros::NodeHandle n("~");

  event_viewer::EventViewer view_events(n);
  
  ros::spin();

  return 0;
}
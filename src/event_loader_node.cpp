#include "dvs_flow/event_loader.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

int main(int argc, char **argv)
{

  ROS_INFO("Event Listener Started");
  ros::init(argc, argv, "event_loader");

  ros::NodeHandle n("~");

  event_loader::EventLoader load_events(n);
  ros::spin();

  return 0;
}
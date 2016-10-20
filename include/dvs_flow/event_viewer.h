#ifndef EVENT_VIEWER_H_
#define EVENT_VIEWER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>

namespace event_viewer
{

class EventViewer {
public:
  EventViewer(ros::NodeHandle &nh);
  virtual ~EventViewer();

private:
  std::vector<dvs_msgs::Event> events;

  cv::Mat image_;
  bool flag_image_used_; //why do we need this ?

  ros::NodeHandle nh_;

  ros::Subscriber events_sub_;

  image_transport::Subscriber image_sub_;
  image_transport::Publisher events_pub_;

  void eventListenerCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void imageListenerCallback(const sensor_msgs::Image::ConstPtr& msg);

};
} // namespace

#endif // EVENT_VIEWER_H_

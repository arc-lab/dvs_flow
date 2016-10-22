#ifndef event_loader_H_
#define event_loader_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <vector> 
#include "compute_flow/compute_flow.h"

namespace event_loader
{

typedef std::vector<dvs_msgs::Event> dvs_events;
class EventLoader {
public:
  EventLoader(ros::NodeHandle &nh);
  virtual ~EventLoader();

private:
  dvs_events events_;
  std::vector<dvs_events> events_array_;

  cv::Mat image_;
  bool flag_image_used_; //why do we need this ?

  ros::NodeHandle nh_;

  ros::Subscriber events_sub_;

  image_transport::Subscriber image_sub_;
  image_transport::Publisher events_pub_;

  void eventListenerCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void imageListenerCallback(const sensor_msgs::Image::ConstPtr& msg);
  void displayEvents(std::vector<dvs_events> &e_array);

};
} // namespace

#endif // event_loader_H_

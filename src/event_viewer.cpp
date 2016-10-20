#include "dvs_flow/event_viewer.h"
namespace event_viewer{

  EventViewer::EventViewer(ros::NodeHandle &nh):nh_(nh)
  {
      //initialize subscribers and publishers
    ROS_INFO("Subscribing to \"events\" Started");
    events_sub_ = nh_.subscribe("/dvs/events",1000, &EventViewer::eventListenerCallback,this);  

    image_transport::ImageTransport it_(nh_);
    image_sub_ = it_.subscribe("/dvs/image_raw", 1, &EventViewer::imageListenerCallback, this);

    cv::namedWindow("view");  //ADD FLAG TO RUN FOLLOWING -- NOT REQUIRED USED FOR DEBUGING
    cv::startWindowThread();  //ADD FLAG TO RUN FOLLOWING -- NOT REQUIRED USED FOR DEBUGING
    cv::waitKey(30);  //ADD FLAG TO RUN FOLLOWING -- NOT REQUIRED USED FOR DEBUGING
      //TODO:: initialize events queue
  }

  EventViewer::~EventViewer()
  {

      cv::destroyWindow("view"); //ADD FLAG TO RUN FOLLOWING -- NOT REQUIRED USED FOR DEBUGING
      //TODO::clean shutdown publishers and subscribers 
  }

  void EventViewer::eventListenerCallback(const dvs_msgs::EventArray::ConstPtr& msg)
  {
      ROS_INFO_STREAM("No of Events Received: "<<msg->events.size()); 
      //TODO Populate event data structure
  }

  void EventViewer::imageListenerCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
      ROS_INFO_STREAM("image listener to be implemented"); 

       //Read the image, and populate image Mat
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image); //ADD FLAG TO RUN FOLLOWING -- NOT REQUIRED USED FOR DEBUGING
        cv::waitKey(30); //ADD FLAG TO RUN FOLLOWING -- NOT REQUIRED USED FOR DEBUGING
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      /*if (!used_last_image_)
      {
        cv_bridge::CvImage cv_image;
        last_image_.copyTo(cv_image.image);
        cv_image.encoding = "bgr8";
        std::cout << "publish image from callback" << std::endl;
        image_pub_.publish(cv_image.toImageMsg());
      }
      used_last_image_ = false;*/

     
  }


}
#include "dvs_flow/event_loader.h"
namespace event_loader{

  EventLoader::EventLoader(ros::NodeHandle &nh):nh_(nh)
  {
      //initialize subscribers and publishers
    ROS_INFO("Subscribing to \"events\" Started");
    events_sub_ = nh_.subscribe("/dvs/events",1000, &EventLoader::eventListenerCallback,this);  

    image_transport::ImageTransport it_(nh_);
    image_sub_ = it_.subscribe("/dvs/image_raw", 1, &EventLoader::imageListenerCallback, this);

    cv::namedWindow("view");  //ADD FLAG TO RUN FOLLOWING -- NOT REQUIRED USED FOR DEBUGING
    cv::startWindowThread();  //ADD FLAG TO RUN FOLLOWING -- NOT REQUIRED USED FOR DEBUGING
    cv::waitKey(30);  //ADD FLAG TO RUN FOLLOWING -- NOT REQUIRED USED FOR DEBUGING
      //TODO:: initialize events queue
  }

  EventLoader::~EventLoader()
  {
      
      cv::destroyWindow("view"); //ADD FLAG TO RUN FOLLOWING -- NOT REQUIRED USED FOR DEBUGING
      //TODO::clean shutdown publishers and subscribers 
  }

  void EventLoader::eventListenerCallback(const dvs_msgs::EventArray::ConstPtr& msg)
  {
      ROS_INFO_STREAM("No of Events Received: "<<msg->events.size()); 
      
      //FIND A BETTER WAY TO DO IT!
      for ( auto it = msg->events.begin(); it != msg->events.end(); ++it )
          events_.push_back(*it);
      ROS_INFO_STREAM("Events Recorded "<<events_.size());
      //TODO Populate event data structure
  }

  void EventLoader::imageListenerCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
      //ROS_INFO_STREAM("image listener to be implemented"); 

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
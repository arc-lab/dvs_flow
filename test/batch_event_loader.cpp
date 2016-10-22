#include "dvs_flow/event_loader.h"
const int BATCH_SIZE = 3000;
namespace event_loader{

  EventLoader::EventLoader(ros::NodeHandle &nh):nh_(nh)
  {
      //initialize subscribers and publishers
    ROS_INFO("Subscribing to \"events\" Started");
    events_sub_ = nh_.subscribe("/dvs/events",1000, &EventLoader::eventListenerCallback,this);  

    image_transport::ImageTransport it_(nh_);
    image_sub_ = it_.subscribe("/dvs/image_raw", 1, &EventLoader::imageListenerCallback, this);
    dvs_events e;
    events_array_.push_back(e); //initialize empty event and push back to events_aray
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
    ROS_INFO_STREAM("Fresh Events Received: "<<msg->events.size()); 
    ROS_INFO_STREAM("events_array_ size"<<events_array_.size());
    if(events_array_[0].size()<BATCH_SIZE)
    {
      events_array_[0].insert(events_array_[0].end(),msg->events.begin(),msg->events.end());      
      ROS_INFO_STREAM("Events Recorded [0]"<<events_array_[0].size());
    }
    else
    {
      if(events_array_.size() == 1)
      {
        dvs_events e;
        events_array_.push_back(e);
        ROS_INFO_STREAM("New Event Created");
      }
      events_array_[1].insert(events_array_[1].end(),msg->events.begin(),msg->events.end()); 
      ROS_INFO_STREAM("Events Recorded "<<events_array_[1].size());
    }
    compute_flow::compute_flow();
      //displayEvents(events_array_);
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

  void EventLoader::displayEvents(std::vector<dvs_events> &e)
  {
      for ( int i = 0; i < e.size(); ++i )
      {
        for (int j = 0; j < e[i].size() ; ++j)
        {
          ROS_INFO_STREAM(" x ["<<i<<"]["<<j<<"[= "<<e[i][j].x);
        }
      }
      system("pause");
  }
}
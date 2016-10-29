#include "dvs_flow/event_loader.h"
const int BATCH_SIZE = 3000;
namespace event_loader{

  EventLoader::EventLoader(ros::NodeHandle &nh):nh_(nh)
  {
      //initialize subscribers and publishers
    ROS_INFO("Subscribing to \"events\" Started");
    events_sub_ = nh_.subscribe("/dvs/events",1000, &EventLoader::eventListenerCallback,this);  
    events_pub_= nh_.advertise<pcl::PointCloud<PointT>> ("events", 10);
    events_pcl_.header.frame_id = "odom";
    events_pcl_.width = 0;
    events_pcl_.height = 1;
    

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
    //ROS_INFO_STREAM("events_array_ size"<<events_array_.size());
    if(events_pcl_.size()<BATCH_SIZE)
    {
      //events_array_[0].insert(events_array_[0].end(),msg->events.begin(),msg->events.end());      
      // events_pcl_.width += msg->events.size();
      // events_pcl_.data.insert(events_array_[0].end(),msg->events.begin(),msg->events.end()); 
      for(int i = 0; i < msg->events.size();++i)
      {
       PointT f_event;
       f_event.x = msg->events[i].x/10;
       f_event.y = msg->events[i].y/10;
       f_event.z = (msg->events[i].ts.toSec() - msg->events[0].ts.toSec())*100;
      //ROS_INFO_STREAM("Z = "<<f_event.z);
       f_event.intensity = (msg->events[i].polarity?0:1) * 255;
       events_pcl_.push_back(f_event);
      }
      ROS_INFO_STREAM("Events Recorded in PCL"<<events_pcl_.size());
      
    }
    if(events_pcl_.size()>=BATCH_SIZE)
    {
      ROS_INFO_STREAM(" --- Events Recorded in PCL"<<events_pcl_.size());
      events_pcl_.header.stamp = ros::Time::now().toNSec();
      events_pub_.publish(events_pcl_);
      ROS_INFO_STREAM("PUBLISHED POINT_CLOUD ");
      //compute_flow::compute_flow();
      events_pcl_.clear();
      events_pcl_.height = 1;
      ROS_INFO_STREAM(" -- Events Erased"<< events_pcl_.size());
    }

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
  }
}
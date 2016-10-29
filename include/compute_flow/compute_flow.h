#ifndef compute_flow_H_
#define compute_flow_H_

#include <iostream>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "ros/ros.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Headers for extractNormals
#include <pcl/features/normal_3d.h>

// Headers for fit plane
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace compute_flow
{
		typedef pcl::PointXYZI PointT;
		void computeFlow(pcl::PointCloud<PointT> &cloud);
		void pcaModified();
		void robustSVD();
		void fitPlane(pcl::PointCloud<PointT> &cloud);
		void extractNormals(pcl::PointCloud<PointT> cloud);
} 

#endif //_compute_flow_H_ 
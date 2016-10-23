#ifndef compute_flow_H_
#define compute_flow_H_

#include <iostream>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "ros/ros.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace compute_flow
{
		typedef pcl::PointXYZI PointT;
		void compute_flow(pcl::PointCloud<PointT> cloud);
		void pca_modified();
		void robust_svd();
		void fit_plane(const pcl::PointCloud<PointT>::Ptr &cloud);
} 

#endif //_compute_flow_H_ 
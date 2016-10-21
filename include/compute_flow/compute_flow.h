#ifndef compute_flow_H_
#define compute_flow_H_

#include <iostream>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "ros/ros.h"

namespace compute_flow
{
		void compute_flow(/** **/);
		void pca_modified();
		void robust_svd();
		void fit_plane();
} 

#endif //_compute_flow_H_ 
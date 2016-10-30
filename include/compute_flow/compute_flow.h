#ifndef compute_flow_H_
#define compute_flow_H_

#include <iostream>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

namespace compute_flow
{
	const int NROWS = 340, NCOLS = 340; //Max and Min Value of X and Y (Event Positions)
	const int N = 5; // TimeStamp Patch Mask Dimension around the Events
	const double TIME_THRES = 0.2;
	const double TH2 = 0.1;
	typedef pcl::PointXYZI PointT;
	typedef Eigen::MatrixXd Mat;
	
	//Eigen::MatrixXd pclToMatrix(pcl::PointCloud<PointT>::Ptr cloud);
	void computeFlow(pcl::PointCloud<PointT>::Ptr cloud);
	bool fitPlane(Mat &m,Eigen::Vector3d &normal, const double TH2);
	void pca(Mat &X,Mat &U);

	void find(Mat &m, Mat &X);
	Mat getPatch(Mat &m,int x, int y, int N);
	Mat getBlock(Mat &m,int x,int y,int N);

	
	struct CWiseThres {
		CWiseThres(const double val,const double thres) : val_(val),thres_(thres){}
		const double operator()(const double& x) const { 
			if(abs(x - val_)/val_ > thres_)
				return 0.0;
			else
				return x;
			 };
		double val_, thres_;
	};
} 

#endif //_compute_flow_H_ 
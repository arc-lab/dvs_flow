#include "compute_flow/compute_flow.h"

namespace compute_flow
{
	//[vx, vy, It] = computeFlow(x, y, t, pol, N, TH1, TH2, NCOLS, NROWS)
	void computeFlow(pcl::PointCloud<PointT>::Ptr events_ptr)
	{


		ROS_INFO_STREAM("compute_flow()");
		
		// Mat m = Mat::Random(5,6);
		// getPatch(m,3,3,2);
		

		/** CONVERT THIS FUNCTION
		**/

		Mat img_ts_pos = Mat::Zero(NROWS, NROWS); //Matrix of timestamps of last events with positive polarity
		Mat img_ts_neg =  Mat::Zero(NROWS, NCOLS); //Matrix of timestamps of last events with negative polarity
		Mat vx= Mat::Zero(NROWS,NCOLS);
		Mat vy= Mat::Zero(NROWS,NCOLS);
		Mat m;
		
		
		std::cout <<"mask timestampe pos" <<m<< std::endl;

		for(auto i= 0; i < events_ptr->size(); ++i)
		{
			int x = events_ptr->points[i].x;
			int y = events_ptr->points[i].y;
			double ts = events_ptr->points[i].z;

			if(events_ptr->points[i].intensity == 255)
			{
				std::cout<<"Patch from Positive Polarity"<<std::endl;
				img_ts_pos(x,y) = ts;
				m = getBlock(img_ts_pos,x,y,N);
			}
			else if(events_ptr->points[i].intensity == 0)
			{
				std::cout<<"Patch from Negative Polarity"<<std::endl;
				img_ts_neg(x,y) = ts;
				m = getBlock(img_ts_neg,x,y,N);
			}
			else
			{
				ROS_ERROR("Polarity of the events NOT GOOD");
				exit (EXIT_FAILURE);
			}

			if(m.size()!=0)
			{
				//Select Events that are close in the time_stamp
				Eigen::Vector3d normal;
				if(fitPlane(m,normal,TH2))
				{
				std::cout<<"Normal computed "<<normal;
				double vx = -1*normal(2)/(std::pow(normal(1),2)+std::pow(normal(0),2))*normal(0)*1e6;
				double vy = -1*normal(2)/(std::pow(normal(1),2)+std::pow(normal(0),2))*normal(1)*1e6;
				std::cout<<"vx = "<<vx<<"vy = "<<vy<<std::endl;
				std::cout<<"Press any key to continue";
				std::cin.ignore(); 
				}
			}
			else
			{
				std::cout<<"NULL MATRIX"<<std::endl;
			}
			//Mat m = Mat::Zero(2*N+1,2*N+1); //Time Stamp patch 

		}
	}

	void find(Mat &m, Mat &X)
	{
		int cnt = 0;
		for(int i = 0; i < m.rows(); ++i)
		{
			for(int j = 0; j < m.cols(); ++j)
			{	
				if(m(i,j)!=0)
				{	
					Eigen::Vector3d vec(j,i,m(i,j)); //Check for X and Y Values
					//std::cout<<" Vector = "<<vec<<std::endl;
					X.conservativeResize(X.rows() + 1, Eigen::NoChange);
					X.row(X.rows()-1) = vec;
					//std::cout<<"X  = "<<X<<std::endl;
				}
			}
		}
	}

	bool fitPlane(Mat &m, Eigen::Vector3d &normal, const double TH2)
	{
		double c_val = m(N,N);
		// std::cout<<"central val = "<<c_val<<std::endl;
		// std::cout<<"Before thresholding \n"<<m<<std::endl;
		m = m.unaryExpr(CWiseThres(c_val,TIME_THRES));
		// std::cout<<"After thresholding \n"<<m<<std::endl;
		if(m.sum() > 0)
		{
			Mat X(0,3);
			Mat U;
			find(m,X);
			ROS_INFO_STREAM("fit plane input \n"<<X);
			if(X.rows() >= 3)
			{
			pca(X,U);
			normal= U.col(2);
			std::cout<<"computed Normal"<< normal<<std::endl;
			return true;
			}
			else
				return false;
		}


	}
	//function [U,mu,vars] = pca( X )
	void pca(Mat &X, Mat &U)
	{
		if(X.rows() >= 1) //Checking if there is sufficient data to get coeff > 3
		{
		X = (X.rowwise() - X.colwise().mean())/std::sqrt(X.rows()-1);
		std::cout<<"X_centered"<<X<<std::endl;
		X = X.transpose()*X;
		Eigen::JacobiSVD<Mat> svd(X, Eigen::ComputeFullU | Eigen::ComputeFullV);
		std::cout << "U: " << svd.matrixU().size() << std::endl;
		Mat S = svd.singularValues().asDiagonal();
		std::cout<<"S: "<< S <<std::endl; 
		std::cout << "V: " << svd.matrixV() << std::endl;
		//U = X * svd.matrixV() * S.inverse(); //check if diagnol(1/S) works
		U = svd.matrixV();
		std::cout<<"Checking orthonormality"<<U.transpose()*U<<std::endl;
		ROS_INFO_STREAM("pca_modified()");
		}
	}

	Mat getPatch(Mat &m,int x, int y, int N)
	{
		//std::cout<<m<<std::endl;
		Mat patch;
		int s_x = x-N; //start x index
		int s_y = y-N; //start y index
		int e_x = x + N; //end x index
		int e_y = y + N; //end y index
		int m_x = m.rows()-1;
		int m_y = m.cols()-1;
		s_x = std::max(s_x,0);
		s_y = std::max(s_y,0);
		e_x = std::min(e_x,m_x-1);
		e_y = std::min(e_y,m_y-1);
		

		patch = m.block(s_x,s_y,e_x-s_x+1,e_y-s_y+1);
		// DEBUGGING
		// std::cout<<"x = "<<x<<" y = "<<y<<" max_x = "<<m.rows()<<" max_y = "<<m.cols()<<std::endl; 
		// std::cout<<"dx = "<<e_x-s_x+1<<" dy = "<<e_y-s_y+1<<std::endl;
		// std::cout<<"\npatch = \n"<<patch<<std::endl;
		return patch;
	}

	Mat getBlock(Mat &m,int x, int y, int N)
	{
		Mat patch;
		int L = (2*N)+1;
    	int s_x = x-N; //start x index
  	 	int s_y = y-N; //start y index
  	 	int e_x = x + L; //end x index
 		int e_y = y + L; //end y index

 		if(s_x < 0 || s_y < 0 || e_x >= m.rows() || e_y >= m.cols())
  		{
   	   		return patch; //return patch of zero size
  		}

 		patch = m.block(x-N,y-N,L,L);

 		ROS_INFO_STREAM("\npatch = \n"<<patch<<"\n");
 		return patch;
	}	

}
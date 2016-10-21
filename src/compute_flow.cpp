#include "compute_flow/compute_flow.h"

namespace compute_flow
{
	//[vx, vy, It] = computeFlow(x, y, t, pol, N, TH1, TH2, NCOLS, NROWS)
	void compute_flow()
	{
		ROS_INFO_STREAM("compute_flow()");
		/** CONVERT THIS FUNCTION

	    It_pos=zeros(NROWS,NCOLS);
	    It_neg=zeros(NROWS,NCOLS);
	    vx=zeros(NROWS,NCOLS); vy=zeros(NROWS,NCOLS);
	    for ii=1:1:length(t) 
	    {
	       ptx=x(ii)+1;
	       pty=y(ii)+1;   
		  
		    if pol(ii)==1
			{	  
	        	It_pos(pty,ptx)=t(ii);
		    	m=It_pos(max(pty-N,1):min(pty+N, NROWS),max(ptx-N,1):min(ptx+N, NCOLS));
		    }
		    else
		    {
		        It_neg(pty,ptx)=t(ii);
		        m=It_neg(max(pty-N,1):min(pty+N, NROWS),max(ptx-N,1):min(ptx+N, NCOLS));
		    }
		    
		    if numel(m) == (2*N+1)*(2*N+1)
		    {
				m(abs(m(N+1,N+1)-m)/m(N+1,N+1)>TH1)=0;
		   	    if (sum(m(:)>0))
			    {
			    	
			    	[vvx,vvy]=fitplane(m, TH2);
			**/
					fit_plane();
			/**    	
			    	if(isnan(vvx) || isinf(vvx))
			    	{
			    		vvx = 0;
			    	}
			    	
			    	if(isnan(vvy) || isinf(vvy))
			    	{ 
			    		vvy = 0;
			        }

			        if (norm([vvx,vvy])>0)
			        {
			                    aa=[vvx vvy];
			                    vy(pty,ptx)=aa(1);
			                    vx(pty,ptx)=aa(2);
			        }
	   			}
			}
		}  
	   It = max(cat(3, It_pos, It_neg), [], 3);
		 **/
	}

	//function [vx,vy]=fitplane(mm, TH)
	void fit_plane()
	{
		ROS_INFO_STREAM("fit_plane()");
		/*vx = 0; vy = 0;
		[XX,YY]=find(mm>0);
		X=[];
		for i=1:length(XX)
		{
			X=[X; XX(i) YY(i) mm(XX(i),YY(i))];
		}
		

		[coeff]=pca_modified(X');
		*/
			pca_modified();
		/*	
		if size(coeff,2)< 3 
		{
			return
		}
		
		normal = coeff(:,3);

		vx = -normal(3)/(normal(2)^2+normal(1)^2)*normal(1)*1e6;
		vy = -normal(3)/(normal(2)^2+normal(1)^2)*normal(2)*1e6;
		*/
	}


	//function [U,mu,vars] = pca( X )
	void pca_modified()
	{
		ROS_INFO_STREAM("pca_modified()");
		/*
		d=size(X); n=d(end); d=prod(d(1:end-1));
		if(~isa(X,'double'))
		{
			X=double(X);
		}

		if(n==1)
		{
			mu=X;
			U=zeros(d,1);
			vars=0;
			return;
		}

		mu = mean(X, ndims(X));
		X = bsxfun(@minus,X,mu)/sqrt(n-1);
		X = reshape( X, d, n );

		m=2500;

		if(min(d,n)>m)
		{
			X=X(:,randperm(n,m));
			n=m;
		}

		if(0)
		{
			[U,S]=svd(X,'econ');
			vars=diag(S).^2;
		}
		elseif( d>n )
		{
			[~,SS,V]= robustSvd(X'*X); 
			**/
			robust_svd();
			/**
			vars=diag(SS);
			U = X * V * diag(1./sqrt(vars));
		}
		else
		{
			[~,SS,U]=robustSvd(X*X');
			**/
			robust_svd();
			/**
			vars=diag(SS);
		}

		K=vars>1e-30;
		vars=vars(K);
		U=U(:,K);
		*/
	}

	//function [U,S,V] = robustSvd( X, trials )
	void robust_svd()
	{
		ROS_INFO_STREAM("robust_svd()");
		/**
		if(nargin<2)
		{
			trials=100;
		}

		try
		{
			[U,S,V] = svd(X)
		}
		catch
		{
			if(trials<=0)
			{
				error('svd did not converge');
			}
			n=numel(X);
			j=randi(n);
			X(j)=X(j)+eps;
			[U,S,V]=robustSvd(X,trials-1);
		}
		**/
	}

}
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, SOH DE LOONG.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "pca.h"

void pca(const sensor_msgs::PointCloudConstPtr& cloud, double norm_hist_1[][7], double norm_hist_2[][5])
{
	no_of_points = cloud->points.size();
	double zero_mean_x[no_of_points];
	double zero_mean_y[no_of_points];
	double zero_mean_z[no_of_points];
	double sum_x = 0;
	double sum_y = 0;
	double sum_z = 0;
	//step 1: find mean of cloud
	//ROS_INFO("%d", cloud->points.size());
	for(unsigned int i = 0; i < cloud->points.size(); i++)
	{
	  if(cloud->points[i].x > -10000)
	  {
	    sum_x = sum_x + cloud->points[i].x;
	    sum_y = sum_y + cloud->points[i].y;
	    sum_z = sum_y + cloud->points[i].z;
	  }
	}
	double mean_x = sum_x / cloud->points.size();
	double mean_y = sum_y / cloud->points.size();		
	double mean_z = sum_z / cloud->points.size();
	
	//step 2: subtract mean from cloud to get a new dataset with mean 0
	for(unsigned int j = 0; j < cloud->points.size(); j++)
	{
	    zero_mean_x[j] = cloud->points[j].x - mean_x;
	    zero_mean_y[j] = cloud->points[j].y - mean_y;
	    zero_mean_z[j] = cloud->points[j].z - mean_z;
	}

	//step 3: calculate covariance matrix
	double cov_mat[3][3];
	cov_mat[0][0] = cov(zero_mean_x, zero_mean_x, cloud->points.size());
	cov_mat[0][1] = cov_mat[1][0] = cov(zero_mean_x, zero_mean_y, cloud->points.size());	
	cov_mat[0][2] = cov_mat[2][0] = cov(zero_mean_x, zero_mean_z, cloud->points.size());
	cov_mat[1][1] = cov(zero_mean_y, zero_mean_y, cloud->points.size());
	cov_mat[1][2] = cov_mat[2][1] = cov(zero_mean_y, zero_mean_z, cloud->points.size());	
	cov_mat[2][2] = cov(zero_mean_z, zero_mean_z, cloud->points.size());

	//step 4: find eigenvectors and eigenvalues of covariance matrix
	double eigen_value[3];
	double eigen_vectors[3][3];	
	jacobi(cov_mat, 3, eigen_value, eigen_vectors);
	//sort the eigenvectors in decending eigen values
	eigsrt(eigen_value, eigen_vectors, 3);
	//step 5 & final step of pca: get new datasets along e1, e2 & e1, e3
	double eigen_dir_12[2][no_of_points];
	double eigen_dir_13[2][no_of_points];

	for(unsigned int k = 0; k < cloud->points.size(); k++)
	{
	    for(int l = 0; l < 2; l++)
	    {
		eigen_dir_12[l][k] = zero_mean_x[k]*eigen_vectors[0][l] + zero_mean_y[k] 			*eigen_vectors[1][l] + zero_mean_z[k]*eigen_vectors[2][l]; 	
		
		if(l == 0)
		{
		    eigen_dir_13[l][k] = zero_mean_x[k]*eigen_vectors[0][l] + zero_mean_y[k] 			    *eigen_vectors[1][l] + zero_mean_z[k]*eigen_vectors[2][l]; 	
		}
		else if(l == 1)
		{
		    eigen_dir_13[l][k] = zero_mean_x[k]*eigen_vectors[0][l+1] + zero_mean_y[k]   		    	    *eigen_vectors[1][l+1] + zero_mean_z[k]*eigen_vectors[2][l+1]; 	
		}
	    }
	}

	//compute normalized 2d histogram
	//calculate the histogram of 14 x 7 bins along e1, e2 & histogram of 9 x 5
	//all in the same loop to save computational time
	double hist_1[14][7] = {0};	
	double hist_2[9][5] = {0};	
	int total_points_1 = 0;
	int total_points_2 = 0;

	for(unsigned int m = 0; m < cloud->points.size(); m++)
	{
		for(int n = 0; n < 14; n++)
		{
			for(int o = 0; o < 7; o++)
			{
				if(eigen_dir_12[0][m] <= (1 - (n * 0.15))       &&
				   eigen_dir_12[0][m] >= (1 - ((n + 1) * 0.15)) &&
				   eigen_dir_12[1][m] >= (-0.5 + (o * 0.15))    &&
				   eigen_dir_12[1][m] <= (-0.5 + ((o + 1) * 0.15)))
				{
					total_points_1++;				
					hist_1[n][o] = hist_1[n][o] + 1; 
				}
				if(n < 9 && o < 5)
				{
					if(eigen_dir_13[0][m] <= (1 - (n * 0.25))    &&
				   	eigen_dir_13[0][m] >= (1 - ((n + 1) * 0.25)) &&
				   	eigen_dir_13[1][m] >= (-0.5 + (o * 0.25))    &&
				   	eigen_dir_13[1][m] <= (-0.5 + ((o + 1) * 0.25)))
					{
						total_points_2++;
						hist_2[n][o] = hist_2[n][o] + 1;
					}
				}					
			}
		}
	}

	for(int s = 0; s < 14; s++)
	{
		for(int t = 0; t < 7; t++)
		{
			norm_hist_1[s][t] = hist_1[s][t] / total_points_1;
			
			if(s < 9 && t < 5)
			{
				norm_hist_2[s][t] = hist_2[s][t] / total_points_2;
			}
		}
	}
}

void pca(sensor_msgs::PointCloud cloud, double norm_hist_1[][7], double norm_hist_2[][5])
{
	no_of_points = cloud.points.size();
	double zero_mean_x[no_of_points];
	double zero_mean_y[no_of_points];
	double zero_mean_z[no_of_points];
	double sum_x = 0;
	double sum_y = 0;
	double sum_z = 0;
	//step 1: find mean of cloud
	for(unsigned int i = 0; i < cloud.points.size(); i++)
	{
	  if(cloud.points[i].x > -10000)
	  {
	    sum_x = sum_x + cloud.points[i].x;
	    sum_y = sum_y + cloud.points[i].y;
	    sum_z = sum_z + cloud.points[i].z;
	  }
	}
	double mean_x = sum_x / cloud.points.size();
	double mean_y = sum_y / cloud.points.size();		
	double mean_z = sum_z / cloud.points.size();
	
	//step 2: subtract mean from cloud to get a new dataset with mean 0
	for(unsigned int j = 0; j < cloud.points.size(); j++)
	{
	    zero_mean_x[j] = cloud.points[j].x - mean_x;
	    zero_mean_y[j] = cloud.points[j].y - mean_y;
	    zero_mean_z[j] = cloud.points[j].z - mean_z;
	}

	//step 3: calculate covariance matrix
	double cov_mat[3][3];
	cov_mat[0][0] = cov(zero_mean_x, zero_mean_x, cloud.points.size());
	cov_mat[0][1] = cov_mat[1][0] = cov(zero_mean_x, zero_mean_y, cloud.points.size());	
	cov_mat[0][2] = cov_mat[2][0] = cov(zero_mean_x, zero_mean_z, cloud.points.size());
	cov_mat[1][1] = cov(zero_mean_y, zero_mean_y, cloud.points.size());
	cov_mat[1][2] = cov_mat[2][1] = cov(zero_mean_y, zero_mean_z, cloud.points.size());	
	cov_mat[2][2] = cov(zero_mean_z, zero_mean_z, cloud.points.size());

	//step 4: find eigenvectors and eigenvalues of covariance matrix
	double eigen_value[3];
	double eigen_vectors[3][3];	
	jacobi(cov_mat, 3, eigen_value, eigen_vectors);
	//sort the eigenvectors in decending eigen values
	eigsrt(eigen_value, eigen_vectors, 3);
	//step 5 & final step of pca: get new datasets along e1, e2 & e1, e3
	double eigen_dir_12[2][no_of_points];
	double eigen_dir_13[2][no_of_points];

	for(unsigned int k = 0; k < cloud.points.size(); k++)
	{
	    for(int l = 0; l < 2; l++)
	    {
		eigen_dir_12[l][k] = zero_mean_x[k]*eigen_vectors[0][l] + zero_mean_y[k] 			*eigen_vectors[1][l] + zero_mean_z[k]*eigen_vectors[2][l]; 	
		
		if(l == 0)
		{
		    eigen_dir_13[l][k] = zero_mean_x[k]*eigen_vectors[0][l] + zero_mean_y[k] 			    *eigen_vectors[1][l] + zero_mean_z[k]*eigen_vectors[2][l]; 	
		}
		else if(l == 1)
		{
		    eigen_dir_13[l][k] = zero_mean_x[k]*eigen_vectors[0][l+1] + zero_mean_y[k]   		    	    *eigen_vectors[1][l+1] + zero_mean_z[k]*eigen_vectors[2][l+1]; 	
		}
	    }
	}

	//compute normalized 2d histogram
	//calculate the histogram of 14 x 7 bins along e1, e2 & histogram of 9 x 5
	//all in the same loop to save computational time
	double hist_1[14][7] = {0};	
	double hist_2[9][5] = {0};	
	int total_points_1 = 0;
	int total_points_2 = 0;

	for(unsigned int m = 0; m < cloud.points.size(); m++)
	{
		for(int n = 0; n < 14; n++)
		{
			for(int o = 0; o < 7; o++)
			{
				if(eigen_dir_12[0][m] <= (1 - (n * 0.15))       &&
				   eigen_dir_12[0][m] >= (1 - ((n + 1) * 0.15)) &&
				   eigen_dir_12[1][m] >= (-0.5 + (o * 0.15))    &&
				   eigen_dir_12[1][m] <= (-0.5 + ((o + 1) * 0.15)))
				{
					total_points_1++;				
					hist_1[n][o] = hist_1[n][o] + 1; 
				}
				if(n < 9 && o < 5)
				{
					if(eigen_dir_13[0][m] <= (1 - (n * 0.25))    &&
				   	eigen_dir_13[0][m] >= (1 - ((n + 1) * 0.25)) &&
				   	eigen_dir_13[1][m] >= (-0.5 + (o * 0.25))    &&
				   	eigen_dir_13[1][m] <= (-0.5 + ((o + 1) * 0.25)))
					{
						total_points_2++;
						hist_2[n][o] = hist_2[n][o] + 1;
					}
				}					
			}
		}
	}

	for(int s = 0; s < 14; s++)
	{
		for(int t = 0; t < 7; t++)
		{
			norm_hist_1[s][t] = hist_1[s][t] / total_points_1;
			
			if(s < 9 && t < 5)
			{
				norm_hist_2[s][t] = hist_2[s][t] / total_points_2;
			}
		}
	}
}

double cov(double x[], double y[], int size)
{
    double result;
    double mul = 0;
    for(int i = 0; i < size; i++)
    {
	mul += x[i]*y[i];
    }
    result = mul / (size - 1);
    return result;   
}

void jacobi(double a[][3], int n, double d[], double v[][3])
{
	int j,iq,ip,i;
	double tresh,theta,tau,t,sm,s,h,g,c;
	double b[3];
	double z[3];	
	for (ip = 0; ip < n; ip++) 
	{
		for (iq = 0; iq < n; iq++) 
			v[ip][iq] = 0.0;
		v[ip][ip] = 1.0;
	}
	for (ip = 0; ip < n; ip++)
 	{
		b[ip] = d[ip] = a[ip][ip];
		z[ip] = 0.0;
	}
	for (i = 0; i < 50; i++) 
	{
		sm = 0.0;
		for (ip = 0; ip < n-1 ; ip++) 
		{
			for(iq = ip+1; iq < n; iq++)
				sm += fabs(a[ip][iq]);
		}
		if(sm == 0.0) 
		{
			return;
		}
		if (i < 4)
			tresh = 0.2*sm/(n*n);
		else
			tresh = 0.0;
		for(ip = 0; ip < n-1; ip++)
		{
			for (iq = ip+1; iq < n; iq++) 
			{
				g=100.0*fabs(a[ip][iq]);
				if (i > 4 && (double)(fabs(d[ip])+g) == (double)fabs(d[ip])
				&& (double)(fabs(d[iq])+g) == (double)fabs(d[iq]))
					a[ip][iq] = 0.0;
				else if (fabs(a[ip][iq]) > tresh) 
				{
					h = d[iq]-d[ip];
					if ((double)(fabs(h)+g) == (double)fabs(h))
						t = (a[ip][iq])/h;
					else 
					{
						theta = 0.5*h/(a[ip][iq]);
						t = 1.0/(fabs(theta)+sqrt(1.0+theta*theta));
						if (theta < 0.0) t = -t;
					}
					c = 1.0/sqrt(1+t*t);
					s = t*c;
					tau = s/(1.0+c);
					h = t*a[ip][iq];
					z[ip] -= h;
					z[iq] += h;
					d[ip] -= h;
					d[iq] += h;
					a[ip][iq] = 0.0;
					for (j = 0; j < ip-1; j++) 
					{
						ROTATE(a,j,ip,j,iq)
					}
					for (j=ip+1;j<iq-1;j++) 
					{
						ROTATE(a,ip,j,j,iq)
					}
					for (j=iq+1;j<n;j++) 
					{
						ROTATE(a,ip,j,iq,j)
					}
					for (j=0;j<n;j++) 
					{
						ROTATE(v,j,ip,j,iq)
					}
					//++(*nrot);
				}
			}
		}
		for (ip = 0; ip < n; ip++) 
		{
			b[ip] += z[ip];
			d[ip] = b[ip];
			z[ip] = 0.0;
		}
	}
}

void eigsrt(double d[], double v[][3], int n)
{
	int k,j,i;
	double p;
	for (i = 0; i < n; i++) 
	{
		p=d[k=i];
		for (j = i+1; j < n; j++)
			if (d[j] >= p) 
				p=d[k=j];
		if (k != i) 
		{
			d[k] = d[i];
			d[i] = p;
			for (j = 0; j < n; j++) 
			{
				p = v[j][i];
				v[j][i] = v[j][k];
				v[j][k] = p;
			}
		}
	}
}


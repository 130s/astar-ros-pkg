#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include <math.h>
#define ROTATE(a,i,j,k,l) g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);\
a[k][l]=h+s*(g-h*tau);

int no_of_points = 10000;
//find covariance
double cov(double x[], double y[], int size);
//jacobi method used to get eigenvalues and eigenvectors
void jacobi(double a[][3], int n, double d[], double v[][3]);
//sort the eigenvectors
void eigsrt(double d[], double v[][3], int n);
//for cloud that is read from callback
void pca(const sensor_msgs::PointCloudConstPtr& cloud, double norm_hist_1[][7], double norm_hist_2[][5]);
//for cloud that is created in the same program
void pca(sensor_msgs::PointCloud cloud, double norm_hist_1[][7], double norm_hist_2[][5]);

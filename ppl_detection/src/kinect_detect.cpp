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

#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "pcl/point_types.h"
#include "pcl_ros/point_cloud.h"
#include "svm.h"
#include "pca.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PolygonStamped.h"
#include "ppl_detection/Tracker.h"

std::string model_name = "my_model"; //change model name if required

#define no_of_attributes 144
#define max_tracker_no 10
#define max_obj_no 200
#define sleep 20
int slp = 0;

void draw_polygon(geometry_msgs::PolygonStamped *poly, float max_x, float min_x, float max_y, float min_y, float max_z, float min_z);

struct object {
  double pos_x;
  double vel_x;
  double pos_y;
  double vel_y;
  double pos_z;
  double vel_z;
  double max_x;
  double max_y;
  double max_z;
  double min_x;
  double min_y;
  double min_z;
  sensor_msgs::PointCloud cloud;
  int ppc;
}obj[max_obj_no];

struct track {
  int text_id;
  double pos_x;
  double pos_y;
  double pos_z;
  double prev_x;
  double prev_y;
  double prev_z;
  double vel_x;
  double vel_y;
  double vel_z;
  bool istracking;
  int stationary;
  int isnonhuman;
}tracker[max_tracker_no];

struct svm_model *model;

bool first = true;
int obj_no = 0;
int next_tracker = 0;
double avg = 0;
int count = 0;
bool start = true;
ros::Time begin;
double svm_predictor(sensor_msgs::PointCloud cloud);

class KinectDetect
{
public:
  ros::NodeHandle n_;
  ros::Subscriber full_sub_;
  ros::Publisher pos_pub_;
  ros::Publisher obj_pub_[max_tracker_no];
  ros::Publisher txt_pub_[max_tracker_no];
  ros::Publisher poly_pub_[max_tracker_no];
  ros::Publisher people_pub_;
  double prev_x, prev_y, prev_z;
  KinectDetect(ros::NodeHandle n) : 
    n_(n)
  {
    full_sub_ = n_.subscribe<pcl::PointCloud<pcl::PointXYZ> > ("/camera/depth/points",1, &KinectDetect::full_cloud_cb, this);
    pos_pub_ = n_.advertise<sensor_msgs::PointCloud> ("k_pos_cloud", 1);
    people_pub_ = n_.advertise<ppl_detection::Tracker> ("people", 1);
    for(int i = 0; i < max_tracker_no; i++)
    {
       std::string name = "obj_cloud_" + boost::lexical_cast<std::string>(i);
       obj_pub_[i] = n_.advertise<sensor_msgs::PointCloud> (name, 1);
       std::string text = "txt_" + boost::lexical_cast<std::string>(i);
       txt_pub_[i] = n_.advertise<visualization_msgs::Marker>(text, 1);
       std::string poly = "poly_" + boost::lexical_cast<std::string>(i);
       poly_pub_[i] = n_.advertise<geometry_msgs::PolygonStamped>(poly, 1);
       tracker[i].istracking = false;
       tracker[i].stationary = 0;
       tracker[i].isnonhuman = 0;
    }
  }  

  void full_cloud_cb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
  {
     obj_no = 0;
     for(unsigned int i = 0; i < cloud->points.size(); i++)
     {
        if(cloud->points[i].y > -1 && cloud->points[i].y < 10 && i%4 == 0)
        {
           if(obj_no == 0)
           {
              obj[0].pos_x = cloud->points[i].x;
              obj[0].pos_y = cloud->points[i].y;
              obj[0].pos_z = cloud->points[i].z;
              obj[0].max_x = cloud->points[i].x;
              obj[0].min_x = cloud->points[i].x;
              obj[0].max_y = cloud->points[i].y;
              obj[0].min_y = cloud->points[i].y;
              obj[0].max_z = cloud->points[i].z;
              obj[0].min_z = cloud->points[i].z;
              obj[0].cloud.points.resize(1);
              obj[0].cloud.points[0].x = cloud->points[i].x;
              obj[0].cloud.points[0].y = cloud->points[i].y;
              obj[0].cloud.points[0].z = cloud->points[i].z;
              obj[0].ppc = 1;
              obj_no = 1;
           }
           else
           {
              bool matched = false;
              for(int j = 0; j < obj_no; j++)
              {
                 if((cloud->points[i].x > obj[j].min_x - 0.1 && cloud->points[i].x < obj[j].max_x + 0.1) &&
                 (cloud->points[i].y > obj[j].min_y - 0.1 && cloud->points[i].y < obj[j].max_y + 0.1) &&
                 (cloud->points[i].z > obj[j].min_z - 0.1 && cloud->points[i].z < obj[j].max_z + 0.1))
                 {
                    obj[j].cloud.points.resize(obj[j].ppc + 1);
                    obj[j].cloud.points[obj[j].ppc].x = cloud->points[i].x;
                    obj[j].cloud.points[obj[j].ppc].y = cloud->points[i].y;
                    obj[j].cloud.points[obj[j].ppc].z = cloud->points[i].z;
                    obj[j].ppc++; 

                    if(cloud->points[i].x > obj[j].max_x)
   	   	       obj[j].max_x = cloud->points[i].x;
                    else if(cloud->points[i].x < obj[j].min_x)
                       obj[j].min_x = cloud->points[i].x;

                    if(cloud->points[i].y > obj[j].max_y)
   		       obj[j].max_y = cloud->points[i].y;
                    else if(cloud->points[i].y < obj[j].min_y)
                       obj[j].min_y = cloud->points[i].y;

                    if(cloud->points[i].z > obj[j].max_z)
		       obj[j].max_z = cloud->points[i].z;
                    else if(cloud->points[i].z < obj[j].min_z)
                       obj[j].min_z = cloud->points[i].z;  

                    obj[j].pos_x = (obj[j].pos_x * (obj[j].ppc - 1) + cloud->points[i].x) / obj[j].ppc;
                    obj[j].pos_y = (obj[j].pos_y * (obj[j].ppc - 1) + cloud->points[i].y) / obj[j].ppc;
                    obj[j].pos_z = (obj[j].pos_z * (obj[j].ppc - 1) + cloud->points[i].z) / obj[j].ppc;

                    matched = true;
                    break;
                  }

              }
          
              if(matched == false && obj_no < max_obj_no - 1)
              {

                 obj[obj_no].pos_x = cloud->points[i].x;
                 obj[obj_no].pos_y = cloud->points[i].y;
                 obj[obj_no].pos_z = cloud->points[i].z;
                 obj[obj_no].max_x = cloud->points[i].x;
                 obj[obj_no].min_x = cloud->points[i].x;
                 obj[obj_no].max_y = cloud->points[i].y;
                 obj[obj_no].min_y = cloud->points[i].y;
                 obj[obj_no].max_z = cloud->points[i].z;
                 obj[obj_no].min_z = cloud->points[i].z;
                 obj[obj_no].cloud.points.resize(1);
                 obj[obj_no].cloud.points[0].x = cloud->points[i].x;
                 obj[obj_no].cloud.points[0].y = cloud->points[i].y;
                 obj[obj_no].cloud.points[0].z = cloud->points[i].z;
                 obj[obj_no].ppc = 1;
                 obj_no++;
              }
           }
        }
     }

     ppl_detection::Tracker people;
     people.header.frame_id = cloud->header.frame_id;
     int no_of_ppl = 0;
     sensor_msgs::PointCloud pos_cloud;
     pos_cloud.header.frame_id = cloud->header.frame_id;

     int cloud_no = 0;
     int orb = 0;
   
     for(int a = 0; a < obj_no; a++)
     {
        if(obj[a].ppc > 8000 && (obj[a].max_x - obj[a].min_x) < 2 && (obj[a].max_y - obj[a].min_y) < 2 && (obj[a].max_z - obj[a].min_z) < 2)
        {
           pos_cloud.points.resize(cloud_no + 1);
           pos_cloud.points[cloud_no].x = obj[a].pos_x;
           pos_cloud.points[cloud_no].y = obj[a].pos_y;
           pos_cloud.points[cloud_no].z = obj[a].pos_z;
           bool tracked = false;
           int tracking_no = 0;
           for(int l = 0; l < max_tracker_no; l++)
           {
              if(tracker[l].pos_x == tracker[l].prev_x &&
                 tracker[l].pos_y == tracker[l].prev_y &&
                 tracker[l].pos_z == tracker[l].prev_z)                 
                 {
                    if(tracker[l].stationary <= 50)
                       tracker[l].stationary++;
                 }

              //if the point has not moved after 50 frames stop tracking
              if(tracker[l].stationary >= 50)
                 tracker[l].istracking = false;

              if(obj[a].pos_x > tracker[l].pos_x - 0.2 &&
                 obj[a].pos_x < tracker[l].pos_x + 0.2 &&
                 obj[a].pos_y > tracker[l].pos_y - 0.2 &&
                 obj[a].pos_y < tracker[l].pos_y + 0.2 &&
                 obj[a].pos_z > tracker[l].pos_z - 0.2 &&
                 obj[a].pos_z < tracker[l].pos_z + 0.2 &&
                 tracker[l].istracking == true && tracked == false)
              {
                 tracked = true;
                 tracking_no = l;
                 tracker[l].stationary = 0;
              }
              tracker[l].vel_x = tracker[l].pos_x - tracker[l].prev_x;
              tracker[l].vel_y = tracker[l].pos_y - tracker[l].prev_y;
              tracker[l].vel_z = tracker[l].pos_z - tracker[l].prev_y;
              tracker[l].prev_x = tracker[l].pos_x;
              tracker[l].prev_y = tracker[l].pos_y;
              tracker[l].prev_z = tracker[l].pos_z;              
              
           }

           visualization_msgs::Marker text_obj;
           text_obj.header.frame_id = cloud->header.frame_id;
           text_obj.ns = "KinectDetect";
           text_obj.id = orb;
           text_obj.type = 9;
           text_obj.action = 0;
           text_obj.pose.position.x = obj[a].pos_x;
           text_obj.pose.position.y = obj[a].pos_y;
           text_obj.pose.position.z = obj[a].pos_z;
  	   text_obj.color.a = 1.0;
           text_obj.scale.x = 0.2;
   	   text_obj.scale.y = 0.2;
  	   text_obj.scale.z = 0.2;
	   text_obj.lifetime = ros::Duration(0.5);
           
           geometry_msgs::PolygonStamped poly;
           poly.header.frame_id = cloud->header.frame_id;

           if(tracked == true)
           {
	      text_obj.color.r = 0.0f; 
	      text_obj.color.g = 1.0f;
     	      text_obj.color.b = 0.0f;
              text_obj.text = "human " + boost::lexical_cast<std::string>(tracking_no + 1);
              tracker[tracking_no].pos_x = obj[a].pos_x;
              tracker[tracking_no].pos_y = obj[a].pos_y;
              tracker[tracking_no].pos_z = obj[a].pos_z;
              draw_polygon(&poly, obj[a].max_x, obj[a].min_x, obj[a].max_y, obj[a].min_y, obj[a].max_z, obj[a].min_z);
              no_of_ppl++;
              people.human.resize(no_of_ppl);
              people.human[no_of_ppl - 1].tracking_no = tracking_no + 1;
              people.human[no_of_ppl - 1].pos_x = obj[a].pos_x;
              people.human[no_of_ppl - 1].pos_y = obj[a].pos_y;
              people.human[no_of_ppl - 1].pos_z = obj[a].pos_z;
              people.human[no_of_ppl - 1].distance = sqrt(obj[a].pos_x * obj[a].pos_x + obj[a].pos_z * obj[a].pos_z);
           }
           else
           {            
              double result = svm_predictor(obj[a].cloud);
	      text_obj.color.r = 1.0f; 
	      text_obj.color.g = 0.0f;
     	      text_obj.color.b = 0.0f;
           
              if(result > 0)
              {
                 text_obj.text = "human";
                 tracker[next_tracker].text_id = orb;
                 tracker[next_tracker].pos_x = obj[a].pos_x;
                 tracker[next_tracker].pos_y = obj[a].pos_y;
                 tracker[next_tracker].pos_z = obj[a].pos_z;
                 tracker[next_tracker].istracking = true;
                 tracker[next_tracker].stationary = 0;
                 draw_polygon(&poly, obj[a].max_x, obj[a].min_x, obj[a].max_y, obj[a].min_y, obj[a].max_z, obj[a].min_z);
                 no_of_ppl++;
                 people.human.resize(no_of_ppl);
                 people.human[no_of_ppl - 1].tracking_no = tracking_no + 1;
                 people.human[no_of_ppl - 1].pos_x = obj[a].pos_x;
                 people.human[no_of_ppl - 1].pos_y = obj[a].pos_y;
                 people.human[no_of_ppl - 1].pos_z = obj[a].pos_z;
                 people.human[no_of_ppl - 1].distance = sqrt(obj[a].pos_x * obj[a].pos_x + obj[a].pos_z * obj[a].pos_z);
                 for(int m = 0; m < max_tracker_no; m++)
                 {
                    if(tracker[m].istracking == false)
                    {
                       next_tracker = m;
                       break;
                    }
                 }
              }
              else
                 text_obj.text = "non-human";
           }

           obj[a].cloud.header.frame_id = cloud->header.frame_id;
           obj_pub_[orb].publish(obj[a].cloud);
           txt_pub_[orb].publish(text_obj);
           poly_pub_[orb].publish(poly);
           orb++;
           cloud_no++;
        }
     }
    
     pos_pub_.publish(pos_cloud);
     people_pub_.publish(people);
     count++;
     begin = ros::Time::now();
     start = false;
  }
};
int main(int argc,char** argv)
{
   ros::init(argc, argv, "kinect_detect");
   ros::NodeHandle n;
   KinectDetect lstopc(n);
   ros::spin();
   return 0;
}

double svm_predictor(sensor_msgs::PointCloud cloud)
{
        struct svm_node node[no_of_attributes];
        double max_height_x = 0;
        double min_height_x = 1000;
        double diff_x;
        double max_height_y = 0;
        double min_height_y = 1000;
        double diff_y;
        double max_height_z = 0;
        double min_height_z = 1000;
        double diff_z;
        double scale = 1;
        //scale image to correct size
        for(int i = 0; i < cloud.points.size(); i++)
        {
            if( cloud.points[i].z > max_height_z)
               max_height_z = cloud.points[i].z;
            if( cloud.points[i].z < min_height_z)
               min_height_z = cloud.points[i].z; 
            if( cloud.points[i].x > max_height_x)
               max_height_x = cloud.points[i].x;
            if( cloud.points[i].x < min_height_x)
               min_height_x = cloud.points[i].x; 
            if( cloud.points[i].y > max_height_y)
               max_height_y = cloud.points[i].y;
            if( cloud.points[i].y < min_height_y)
               min_height_y = cloud.points[i].y; 
        }
        diff_x = max_height_x - min_height_x;
        diff_y = max_height_y - min_height_y;
        diff_z = max_height_z - min_height_z;

        if(diff_x > diff_y && diff_x > diff_z)
           scale = diff_x / 2;
        else if(diff_y > diff_x && diff_y > diff_z)
           scale = diff_y / 2;
        else if(diff_z > diff_x && diff_z > diff_y)
           scale = diff_z / 2;

        for(int j = 0; j < cloud.points.size(); j++)
        {
           cloud.points[j].x =  cloud.points[j].x / scale;
           cloud.points[j].y =  cloud.points[j].y / scale;
           cloud.points[j].z =  cloud.points[j].z / scale;
        }
        //pca
        double norm_hist_1[14][7];	                		         
        double norm_hist_2[9][5];					    
        pca(cloud, norm_hist_1, norm_hist_2);                          
        //add histogram data to an svm_node			           
        int node_index = 0;						    
        int node_index_cont = 98;					    
        for(int i = 0; i < 14; i++)             	        	    
        {								    
          for(int j = 0; j < 7; j++)					   
          {								  
            node[node_index].index = node_index + 1;	                  
            node[node_index].value = norm_hist_1[i][j];                    
            if(i < 9 && j < 5)					        
            {								   
              node[node_index_cont].index = node_index_cont;               
              node[node_index_cont].value = norm_hist_2[i][j];           
              node_index_cont++;					    
            }
            node_index++;
          }
        }
        node[node_index_cont].index = -1;
        //end loop 
        std::string model_string = "svm_models/" + model_name + ".model";
        char *name_model = (char*)model_string.c_str();
        model = svm_load_model(name_model);
        //start prediction of the image
        double dec[10];
        double result = svm_predict_values(model, node, dec);
        return result;
}

void draw_polygon(geometry_msgs::PolygonStamped *poly, float max_x, float min_x, float max_y, float min_y, float max_z, float min_z)
{
   poly->polygon.points.resize(16);

   poly->polygon.points[0].x = min_x;
   poly->polygon.points[0].y = min_y;
   poly->polygon.points[0].z = min_z;

   poly->polygon.points[1].x = min_x;
   poly->polygon.points[1].y = max_y;
   poly->polygon.points[1].z = min_z;

   poly->polygon.points[2].x = min_x;
   poly->polygon.points[2].y = max_y;
   poly->polygon.points[2].z = max_z;

   poly->polygon.points[3].x = min_x;
   poly->polygon.points[3].y = min_y;
   poly->polygon.points[3].z = max_z;

   poly->polygon.points[4].x = max_x;
   poly->polygon.points[4].y = min_y;
   poly->polygon.points[4].z = max_z;

   poly->polygon.points[5].x = max_x;
   poly->polygon.points[5].y = max_y;
   poly->polygon.points[5].z = max_z;

   poly->polygon.points[6].x = min_x;
   poly->polygon.points[6].y = max_y;
   poly->polygon.points[6].z = max_z;

   poly->polygon.points[7].x = max_x;
   poly->polygon.points[7].y = max_y;
   poly->polygon.points[7].z = max_z;

   poly->polygon.points[8].x = max_x;
   poly->polygon.points[8].y = max_y;
   poly->polygon.points[8].z = min_z;

   poly->polygon.points[9].x = min_x;
   poly->polygon.points[9].y = max_y;
   poly->polygon.points[9].z = min_z;

   poly->polygon.points[10].x = max_x;
   poly->polygon.points[10].y = max_y;
   poly->polygon.points[10].z = min_z;

   poly->polygon.points[11].x = max_x;
   poly->polygon.points[11].y = min_y;
   poly->polygon.points[11].z = min_z;

   poly->polygon.points[12].x = max_x;
   poly->polygon.points[12].y = min_y;
   poly->polygon.points[12].z = max_z;

   poly->polygon.points[13].x = min_x;
   poly->polygon.points[13].y = min_y;
   poly->polygon.points[13].z = max_z;

   poly->polygon.points[14].x = min_x;
   poly->polygon.points[14].y = min_y;
   poly->polygon.points[14].z = min_z;

   poly->polygon.points[15].x = max_x;
   poly->polygon.points[15].y = min_y;
   poly->polygon.points[15].z = min_z;

 }

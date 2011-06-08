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
#include "stdlib.h"
#include "ply.h"
#include "svm.h"
#include "pca.h"
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

std::string model_name = "my_model"; //change model name if required
#define no_of_testing_vectors 120    //change number of test vectors 

#define no_of_attributes 144
typedef struct Vertex {
  float x,y,z;            
} Vertex;

char *elem_names[] = { 
  "vertex"
};

PlyProperty vert_props[] = { 
  {"x", PLY_FLOAT, PLY_FLOAT, offsetof(Vertex,x), 0, 0, 0, 0},
  {"y", PLY_FLOAT, PLY_FLOAT, offsetof(Vertex,y), 0, 0, 0, 0},
  {"z", PLY_FLOAT, PLY_FLOAT, offsetof(Vertex,z), 0, 0, 0, 0},
};

double true_positive = 0;
double false_positive = 0;
double true_negative = 0;
double false_negative = 0;
double true_positive_rate[no_of_testing_vectors];
double false_positive_rate[no_of_testing_vectors];
double recall[no_of_testing_vectors];
double precision[no_of_testing_vectors];
     
struct svm_model *model;

void read_ply(char *filename, sensor_msgs::PointCloud& cloud);
class Test
{
public:
  ros::NodeHandle n_;
  ros::Publisher cloud_pub_;

  Test(ros::NodeHandle n) : 
    n_(n)
  {

     int ind = 1;
     //begin loop for every image
     for(int a = 0; a < no_of_testing_vectors; a++)
     {
        int truth;
        std::string file_name;
        if(a%2 == 0)
	{
           truth = 1;
           file_name = "ply/testing/human/" + boost::lexical_cast<std::string>(ind) + ".ply";
	}
        else
	{
           truth = -1;
           file_name = "ply/testing/non_human/" + boost::lexical_cast<std::string>(ind) + ".ply";
           ind++;
	}
        //read ply file and convert them to point clouds
        struct svm_node node[no_of_attributes];
        char *name = (char*)file_name.c_str();
	sensor_msgs::PointCloud ply_cloud;	
	read_ply(name, ply_cloud);
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
        for(int i = 0; i < ply_cloud.points.size(); i++)
        {
            if(ply_cloud.points[i].z > max_height_z)
               max_height_z = ply_cloud.points[i].z;
            if(ply_cloud.points[i].z < min_height_z)
               min_height_z = ply_cloud.points[i].z; 
            if(ply_cloud.points[i].x > max_height_x)
               max_height_x = ply_cloud.points[i].x;
            if(ply_cloud.points[i].x < min_height_x)
               min_height_x = ply_cloud.points[i].x; 
            if(ply_cloud.points[i].y > max_height_y)
               max_height_y = ply_cloud.points[i].y;
            if(ply_cloud.points[i].y < min_height_y)
               min_height_y = ply_cloud.points[i].y; 
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

        for(int j = 0; j < ply_cloud.points.size(); j++)
        {
           ply_cloud.points[j].x = ply_cloud.points[j].x / scale;
           ply_cloud.points[j].y = ply_cloud.points[j].y / scale;
           ply_cloud.points[j].z = ply_cloud.points[j].z / scale;
        }
        //pca
        double norm_hist_1[14][7];	                		         
        double norm_hist_2[9][5];					    
        pca(ply_cloud, norm_hist_1, norm_hist_2);                          
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
        double result = svm_predict(model, node);
        //ROS_INFO("result : %f", result);
        if((int)result == 1 && truth == 1)
        {
	   true_positive = true_positive + 1;
           ROS_INFO("true positive");
        }
        else if((int)result == 1 && truth == -1)
	{
	   false_positive = false_positive + 1;
           ROS_INFO("false positive");
        }
        else if((int)result == -1 && truth == 1)
	{
           false_negative = false_negative + 1;
           ROS_INFO("false negative");
        }
        else if((int)result == -1 && truth == -1)
	{
           true_negative = true_negative + 1;
           ROS_INFO("true negative");
        }
        true_positive_rate[a] = true_positive / (true_positive + false_negative);
        false_positive_rate[a] = false_positive / (false_positive + true_negative);
        recall[a] = true_positive / (true_positive + false_negative);
        precision[a] = true_positive / (true_positive + false_positive);
        ROS_INFO("tpr/recall : %f fpr : %f precision : %f",true_positive_rate[a], false_positive_rate[a],precision[a]);
     }
     ROS_INFO("TP %f FP %f FN %f TN %f", true_positive, false_positive, false_negative, true_negative);
     svm_free_and_destroy_model(&model);

  }  
};
int main(int argc, char** argv)
{
   ros::init(argc, argv, "Test");
   ros::NodeHandle n;
   Test lstopc(n);
   ros::spin();

   return 0;
}

void read_ply(char *filename, sensor_msgs::PointCloud& cloud)
{
  int i,j;
  PlyFile *ply;
  int nelems;
  char **elist;
  int file_type;
  float version;
  int nprops;
  int num_elems;
  PlyProperty **plist;
  Vertex **vlist;
  char *elem_name;

  // open a PLY file for reading 
  ply = ply_open_for_reading(filename, &nelems, &elist, &file_type, &version);

  // go through each kind of element that we learned is in the file 
  // and read them

  for (i = 0; i < nelems; i++) 
  {

    //get the description of the first element
    elem_name = elist[i];
    plist = ply_get_element_description (ply, elem_name, &num_elems, &nprops);

    //if we're on vertex elements, read them in
    if (equal_strings ("vertex", elem_name)) 
    {

      // create a vertex list to hold all the vertices
      vlist = (Vertex **) malloc (sizeof (Vertex *) * num_elems);

      // set up for getting vertex elements

      ply_get_property (ply, elem_name, &vert_props[0]);
      ply_get_property (ply, elem_name, &vert_props[1]);
      ply_get_property (ply, elem_name, &vert_props[2]);

      //grab all the vertex elements 
      cloud.points.resize(num_elems);
      for (j = 0; j < num_elems; j++) 
      {

        //grab and element from the file 
        vlist[j] = (Vertex *) malloc (sizeof (Vertex));
        ply_get_element (ply, (void *) vlist[j]);
	//convert vertex to cloud
	cloud.points[j].x = vlist[j]->x;
	cloud.points[j].y = vlist[j]->y;
	cloud.points[j].z = vlist[j]->z;
	
      }
    }
  }

  ply_close (ply);
}


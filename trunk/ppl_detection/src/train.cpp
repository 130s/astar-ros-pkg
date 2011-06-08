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

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "stdlib.h"
#include "ply.h"
#include "svm.h"
#include "pca.h"
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))


std::string model_name = "my_model";   //change model name of required
#define no_of_training_vectors 1424    //change number of training vectors

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

struct svm_parameter param;
struct svm_model *model;
struct svm_problem problem;
struct svm_node *vectors[no_of_training_vectors];
     
bool read_ply(char *filename, sensor_msgs::PointCloud& cloud);
class Train
{
public:
  ros::NodeHandle n_;
  ros::Publisher cloud_pub_;

  Train(ros::NodeHandle n) : 
    n_(n)
  {
     cloud_pub_ = n_.advertise<sensor_msgs::PointCloud>("ply_cloud",1);
     int vector_no = 0;
     double answer[no_of_training_vectors];
     //loop to scan all ply files
     //start loop
     int ind = 1;
     int ans_count = 0;
     ROS_INFO("Scanning");
     for(int a = 0; a < no_of_training_vectors; a++)
     {
        std::string file_name;
        if(a%2 == 0)
	{
           //answer[a] = 1;
           file_name = "ply/training/human/" + boost::lexical_cast<std::string>(ind) + ".ply";
	}
        else
        {
           //answer[a] = -1;
           file_name = "ply/training/non_human/" + boost::lexical_cast<std::string>(ind) + ".ply";
           ind++;
        }
        char *name = (char*)file_name.c_str();
	sensor_msgs::PointCloud ply_cloud;	
	bool read = read_ply(name, ply_cloud);
        if(read == true)
        {
           int x = ind - 1;
           if(file_name == "ply/training/human/" + boost::lexical_cast<std::string>(ind) + ".ply")
           {
               answer[ans_count] = 1.0;
               ROS_DEBUG("%d answer %f",a,answer[ans_count]);
           }
           else if(file_name == "ply/training/non_human/" + boost::lexical_cast<std::string>(x) + ".ply")
           {
               answer[ans_count] = -1.0;
               ROS_DEBUG("%d answer %f",x,answer[ans_count]);
	   }
           ans_count++;
           vectors[vector_no] = new svm_node[no_of_attributes];
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
           //scale the images to correct size
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
/*        while(ros::ok())
        {
	ply_cloud.header.frame_id = "/world";
	cloud_pub_.publish(ply_cloud);
        }
*/      //pca
           double norm_hist_1[14][7];	                		         
           double norm_hist_2[9][5];					    
           pca(ply_cloud, norm_hist_1, norm_hist_2);                         
        //add histogram data to an svm_node array			   
           int node_index = 0;				
           int node_index_cont = 98;		
           for(int i = 0; i < 14; i++)             	
           {						
             for(int j = 0; j < 7; j++)		
             {						
               vectors[vector_no][node_index].index = node_index + 1;	   
               vectors[vector_no][node_index].value = norm_hist_1[i][j];
               if(i < 9 && j < 5)					        
               {								    
                 vectors[vector_no][node_index_cont].index = node_index_cont;  
                 vectors[vector_no][node_index_cont].value = norm_hist_2[i][j];
                 node_index_cont++;					    
               }
               node_index++;
             }
           }
           vectors[vector_no][node_index_cont].index = -1;
           vector_no++;
        }
     }
     //svm added to the problem file 
     problem.l = vector_no-1;
     problem.y = answer;
     problem.x = vectors;
     param.svm_type = 0;
     param.kernel_type = 2;
     param.degree = 3;// : set degree in kernel function (default 3)\n"
     param.gamma = pow(2, 4);// : set gamma in kernel function (default 1/num_features)\n"
     param.coef0 = 0;// : set coef0 in kernel function (default 0)\n"
     param.C = pow(2, 1);// : set the parameter C of C-SVC, epsilon-SVR, and nu-SVR (default 1)\n"
     param.nu = 0.5;// : set the parameter nu of nu-SVC, one-class SVM, and nu-SVR (default 0.5)\n"
     param.p = 0.1;// : set the epsilon in loss function of epsilon-SVR (default 0.1)\n"
     param.cache_size = 100;// : set cache memory size in MB (default 100)\n"
     param.eps = 0.001;// : set tolerance of termination criterion (default 0.001)\n"
     param.shrinking = 1;// : whether to use the shrinking heuristics, 0 or 1 (default 1)\n"
     param.probability = 1;// : whether to train a SVC or SVR model for probability estimates, 0 or 1 (default 0)\n"
     param.nr_weight = 0;// : set the parameter C of class i to weight*C, for C-SVC (default 1)\n"
     
     ROS_INFO("Saving Model");

     double *target = Malloc(double,problem.l);
     svm_cross_validation(&problem,&param,5,target);
     model = svm_train(&problem, &param);
     std::string model_string = "svm_models/" + model_name + ".model";
     char *name_model = (char*)model_string.c_str();
     int result = svm_save_model(name_model, model);
     if(result == 0)
     {
       ROS_INFO("Model saved");
     }
     else 
       ROS_INFO("Error: Model not saved");
     svm_free_and_destroy_model(&model);
     svm_destroy_param(&param);

     exit(0);
  }  
};
int main(int argc, char** argv)
{
   ros::init(argc, argv, "train");
   ros::NodeHandle n;
   Train lstopc(n);
   ros::spin();

   return 0;
}

bool read_ply(char *filename, sensor_msgs::PointCloud& cloud)
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
  if(ply == NULL)
  {
     return false;
  }
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
  return true;
}


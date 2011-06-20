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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tilting_servo/servoAction.h>
#include <tilting_servo/tilt_signal.h>

double ang;
double prev_ang;
double max_angle;
double min_angle;
double servo_speed;
bool turn_top = false;
bool turn_bottom = false;
bool stop;
void feedbackCb(const tilting_servo::servoFeedbackConstPtr& feedback)
{
	if(feedback->angle < max_angle + 10 && feedback->angle > min_angle - 10)
	{ 		
		prev_ang = ang;
  		ang = feedback->angle;
	} 
}

void activeCb()
{
}

void doneCb(const actionlib::SimpleClientGoalState& state,
            const tilting_servo::servoResultConstPtr& result)
{
}
int main (int argc, char **argv)
{
	
	ros::init(argc, argv, "check_servo");
	ros::param::get("/max_angle", max_angle);
	ros::param::get("/min_angle", min_angle);
	ros::param::get("/servo_speed", servo_speed);
	ros::param::get("/stop", stop);
	ros::NodeHandle n;	
	ros::Publisher tilt_signal_pub = n.advertise<tilting_servo::tilt_signal>("tilt_signal", 1000);		
	
	actionlib::SimpleActionClient<tilting_servo::servoAction> ac("servo", true);

	ac.waitForServer();

	tilting_servo::servoGoal goal;	
	bool begin = true;
	bool started = false;
	
	if(stop == true)
	{
		goal.angle = 180;
		goal.speed = servo_speed;
		ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	}
	
	else			
	{



	while(ros::ok())
	{
		if(begin == true)
		{
			while(started == false)			
			{
				if(ang > max_angle)
				{
					turn_top = true;
					goal.angle = min_angle - 2;
					goal.speed = servo_speed;
					ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);	
				}				
				else if(ang < min_angle)
				{
					turn_bottom = true;
					goal.angle = max_angle + 2;
					goal.speed = servo_speed;
					ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);	
				}				
				else if(ang != prev_ang)
				{
					started = true;
					begin = false;
				}
				else
				{
					goal.angle = max_angle + 2;
					goal.speed = servo_speed;
					ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
				}
				usleep(10000);
			}				
				
		}
		else if(ang > max_angle && ang == prev_ang)
		{
			turn_top = true;
			ac.cancelGoal();
			goal.angle = min_angle - 2;
			goal.speed = servo_speed;
			ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);	
		}
		else if(ang < min_angle && ang == prev_ang)
		{	
			turn_bottom = true;
			ac.cancelGoal();
			goal.angle = max_angle + 2;
			goal.speed = servo_speed;
			ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
		}
		else if(ang > max_angle - 4 && goal.angle == max_angle + 2)
		{
			turn_top = false;
			turn_bottom = false;
			ac.cancelGoal();
			goal.angle = max_angle + 2;
			goal.speed = servo_speed/2;
			ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);	
		}
		else if(ang < min_angle + 2 && goal.angle == min_angle - 2)
		{	
			turn_top = false;
			turn_bottom = false;
			ac.cancelGoal();
			goal.angle = min_angle - 2;
			goal.speed = servo_speed/2;
			ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
		}
		tilting_servo::tilt_signal signal;
		if(turn_top == true)
		{
			signal.signal = 1;
			turn_top = false;
		}
		else if(turn_bottom == true)
		{
			signal.signal = 2;
			turn_bottom = false;
		}
		else 	
			signal.signal = 0;
		signal.header.stamp = ros::Time::now();	
		tilt_signal_pub.publish(signal);
												
		usleep(10000);
	}
	}
	return 0;
}



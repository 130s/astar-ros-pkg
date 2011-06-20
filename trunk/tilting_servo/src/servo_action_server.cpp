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
#include <actionlib/server/simple_action_server.h>
#include <tilting_servo/servoAction.h>
#include "servo.h"


bool start = true;
int baudrate;
int servo_id;
double Comport;
double servo_speed;
double prev_goal = 0;
double prev_speed = 1;
double max_angle;
double min_angle;

std::string port;
class servoAction
{
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<tilting_servo::servoAction> as_;
	std::string action_name_;
	tilting_servo::servoFeedback feedback_;
	tilting_servo::servoResult result_;

public:

	servoAction(std::string name) :
	as_(nh_, name, boost::bind(&servoAction::executeCB, this, _1), false), 	action_name_(name)
	{
		as_.start();
	}
	
	~servoAction(void)
	{
	}

	void executeCB(const tilting_servo::servoGoalConstPtr &goal)
	{
		if(start == true)
		{
			servo_move(Comport, servo_id, goal->angle, goal->speed);
			prev_goal = goal->angle;
			start = false;
		}
		if(as_.isNewGoalAvailable())
			as_.acceptNewGoal();
		if(goal->angle != prev_goal || goal->speed != prev_speed)
		{
		servo_move(Comport, servo_id, goal->angle, goal->speed);
		prev_goal = goal->angle;
		prev_speed = goal->speed;
		}				
		while((as_.isNewGoalAvailable() == false) && ros::ok())
		{	
			feedback_.angle = read_angle(Comport, servo_id);
			if(feedback_.angle >= min_angle - 10 && feedback_.angle <= max_angle + 10)			
			as_.publishFeedback(feedback_);
		}
		
		// check that preempt has not been requested by the client
		if (as_.isPreemptRequested() || !ros::ok())
		{
			// set the action state to preempted
			as_.setPreempted();
		}


	}

};


int main(int argc,char** argv)
{

	ros::init(argc, argv, "servo");
	ros::param::get("/port", port);
	if(port == "/dev/ttyS0")
		Comport = 0;
	else if(port == "/dev/ttyS1")
			Comport = 1;
	else if(port == "/dev/ttyS2")
			Comport = 2;
	else if(port == "/dev/ttyS3")
			Comport = 3;
	else if(port == "/dev/ttyS4")
			Comport = 4;
	else if(port == "/dev/ttyS5")
			Comport = 5;
	else if(port == "/dev/ttyS6")
			Comport = 6;
	else if(port == "/dev/ttyS7")
			Comport = 7;
	else if(port == "/dev/ttyS8")
			Comport = 8;
	else if(port == "/dev/ttyS9")
			Comport = 9;
	else if(port == "/dev/ttyS10")
			Comport = 10;
	else if(port == "/dev/ttyS11")
			Comport = 11;
	else if(port == "/dev/ttyS12")
			Comport = 12;
	else if(port == "/dev/ttyS13")
			Comport = 13;
	else if(port == "/dev/ttyS14")
			Comport = 14;
	else if(port == "/dev/ttyS15")
			Comport = 15;
	else if(port == "/dev/ttyUSB0")
			Comport = 16;
	else if(port == "/dev/ttyUSB1")
			Comport = 17;
	else if(port == "/dev/ttyUSB2")
			Comport = 18;
	else if(port == "/dev/ttyUSB3")
			Comport = 19;
	else if(port == "/dev/ttyUSB4")
			Comport = 20;
	else if(port == "/dev/ttyUSB5")
			Comport = 21;
	ros::param::get("/baudrate", baudrate);
	ros::param::get("/servo_id", servo_id);
	ros::param::get("/servo_speed", servo_speed);	
	ros::param::get("/max_angle", max_angle);
	ros::param::get("/min_angle", min_angle);
	OpenComport(Comport, baudrate);
	servoAction servo(ros::this_node::getName());
	if (!ros::ok())
	{	
	CloseComport(Comport);
	}	
	ros::spin();
		
	return 0;
}








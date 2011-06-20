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

#include "servo.h"

//move servo to a position with a certain speed
void servo_move(int comport, int ID, int angle, int rpm)
{
	byte servo_id = ID;	

	if (angle > upper_limit)
		angle = upper_limit;
	else if (angle < lower_limit)
		angle = lower_limit; 	

	byte angle_h = (angle * 0X3FF) / (256 * 360);
	byte angle_l = ((angle * 0X3FF) / 360) % 256;
	
	byte rpm_h = (rpm * 0X3FF) / (256 * 114);
	byte rpm_l = ((rpm * 0X3FF) / 114) % 256;

	byte chksum = 0XFF - servo_id - 0X07 - 0X03 - 0X1E - angle_l - angle_h - rpm_l - rpm_h;
	
	byte inst[11] = {0XFF, 0XFF, servo_id, 0X07, 0X03, 0X1E, angle_l, angle_h, rpm_l, rpm_h, chksum};
	send_inst(comport, inst, 11);
	usleep(ret_delay);
	byte buf[20] = {0};	
	PollComport(comport, buf, 20);

	return;
}

//send an instruction to servo
void send_inst(int comport, byte inst[], int length)
{
	int i = 0;
	while(i < length)
	{
		SendByte(comport, inst[i]);
		i++;
	}
	return;
}

//read the position of the servo 
double read_angle(int comport, int ID)
{
	byte buf[20] = {0};
	byte chksum = 0XFF - ID - 0X2C;
	byte inst[8] = {0XFF, 0XFF, ID, 0X04, 0X02, 0X24, 0X02, chksum};
	send_inst(comport, inst, 8);
	usleep(ret_delay);
	PollComport(comport, buf, 20);
	int i = 0;	
	while (buf[i]!=0XFF && buf[i + 1]!=0XFF)
	 	i++;
	double ang = ((double)buf[i + 6] * 256 + (double)buf[i + 5]) * 360 / 0X3FF;	
	return ang;
}

//read the speed of the servo
double read_speed(int comport, int ID)
{
	byte buf[10] = {0};
	byte chksum = 0XFF - ID - 0X2A;
	byte inst[8] = {0XFF, 0XFF, ID, 0X04, 0X02, 0X26, 0X02, chksum};
	send_inst(comport, inst, 8);
	usleep(ret_delay);
	PollComport(comport, buf, 10);
	double speed = ((double)buf[6] * 256 + (double)buf[5]) * 114 / 0X3FF;	
	return speed;
}

//read the reply delay time of the servo
int read_ret_dly_time(int comport, int ID)
{
	byte buf[10] = {0};
	byte chksum = 0XFF - ID - 0X0C;
	byte inst[8] = {0XFF, 0XFF, ID, 0X04, 0X02, 0X05, 0X01, chksum};
	send_inst(comport, inst, 8);
	usleep(ret_delay);
	PollComport(comport, buf, 10);
	return buf[5];
}

//read voltage of servo
int read_voltage(int comport, int ID)
{
	byte buf[10] = {0};
	byte chksum = 0XFF - ID - 0X31;
	byte inst[8] = {0XFF, 0XFF, ID, 0X04, 0X02, 0X2A, 0X01, chksum};
	send_inst(comport, inst, 8);
	usleep(ret_delay);
	PollComport(comport, buf, 10);
	return (buf[5]/10);
}

//read temperature of servo in celsius
int read_temp(int comport, int ID)
{
	byte buf[10] = {0};
	byte chksum = 0XFF - ID - 0X32;
	byte inst[8] = {0XFF, 0XFF, ID, 0X04, 0X02, 0X2B, 0X01, chksum};
	send_inst(comport, inst, 8);
	usleep(ret_delay);
	PollComport(comport, buf, 10);
	return buf[5];
} 

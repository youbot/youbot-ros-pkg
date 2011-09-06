/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Sebastian Blumenthal, Alexey Zakharov
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Locomotec nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

/*
 * Many thanks to Kevin Watts for providing the teleop_pr2_keyboard application
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_U 0x75
#define KEYCODE_J 0x6A
#define KEYCODE_I 0x69
#define KEYCODE_K 0x6B
#define KEYCODE_SAPCEBAR 0x20

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

class KeyboardTeleoperation
{
private:

	double translationalSpeed;
	double rotationalSpeed;
	double incrementRate;
	geometry_msgs::Twist baseCommand;

	ros::NodeHandle n;
	ros::Publisher baseCommandPublisher;
	ros::Subscriber baseOdometrySubscriber;
	ros::Time startTimeStamp;
	nav_msgs::Odometry lastReceivedOdometry;
	geometry_msgs::Pose startPose;
	geometry_msgs::Pose stopPose;


	int experimentNumber;
	bool experimentIsRunning;
	double maxTraveledDistance; //[m]

public:

	KeyboardTeleoperation();
	virtual ~KeyboardTeleoperation() { }

	void init();
	void keyboardLoop();

	void startExperiment();
	void stopExperiment();

	void baseOdometryCallback(const nav_msgs::Odometry& odomertry);

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
	tcsetattr(kfd, TCSANOW, &cooked);
	exit(0);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "keyboard_teleop_experiment");

	KeyboardTeleoperation keyboardTeleopExperiment;

	ros::Rate rate(10); //Hz
	ros::AsyncSpinner spinner(4); // Use 4 threads
   	spinner.start();

	signal(SIGINT,quit);
	keyboardTeleopExperiment.keyboardLoop();

	return(0);
}


KeyboardTeleoperation::KeyboardTeleoperation() {
	this->init();
}

void KeyboardTeleoperation::init() {
	baseCommand.linear.x = 0;
	baseCommand.linear.y = 0;
	baseCommand.angular.z = 0;

	baseCommandPublisher = n.advertise<geometry_msgs::Twist>("base_controller/command", 1);
	baseOdometrySubscriber = n.subscribe("base_odometry/odom", 1, &KeyboardTeleoperation::baseOdometryCallback, this); // subscribing for joint states

	ros::NodeHandle n_private("~");

	n_private.param("translationalSpeed", translationalSpeed, 0.1);
	n_private.param("rotationalSpeed", rotationalSpeed, 0.1);
	n_private.param("incrementRate", incrementRate, 0.1);
	n_private.param("maxTraveledDistance", maxTraveledDistance, 1.0);

	experimentNumber = 0;
	experimentIsRunning = false;
}

void KeyboardTeleoperation::keyboardLoop() {
	char c;
	bool dirty=false;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use 'WASD' to translate");
	puts("Use 'QE' to yaw");
	puts("Press 'U' or 'J' to increase or decrease translational speed");
	puts("Press 'I' or 'K' to increase or decrease translational speed");


	while(true) {
		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0) {
			perror("read():");
			exit(-1);
		}

		//baseCommand.linear.x = 0;
		//baseCommand.linear.y = 0;
		//baseCommand.angular.z = 0;
        ROS_INFO("key code %d}\n", c);
		switch(c) {

		/* Move commands */
		case KEYCODE_W:
			baseCommand.linear.x = translationalSpeed;
			dirty = true;
			startExperiment();
			break;
		case KEYCODE_S:
			baseCommand.linear.x = - translationalSpeed;
			dirty = true;
			startExperiment();
			break;
		case KEYCODE_A:
			baseCommand.linear.y = translationalSpeed;
			dirty = true;
			startExperiment();
			break;
		case KEYCODE_D:
			baseCommand.linear.y = - translationalSpeed;
			dirty = true;
			startExperiment();
			break;
		case KEYCODE_Q:
			baseCommand.angular.z = rotationalSpeed;
			dirty = true;
			startExperiment();
			break;
		case KEYCODE_E:
			baseCommand.angular.z = - rotationalSpeed;
			dirty = true;
			startExperiment();
			break;

		/* Configuration commands */
		case KEYCODE_U:
			translationalSpeed += incrementRate;
			ROS_INFO("New translational velocity = %f", translationalSpeed);
			break;
		case KEYCODE_J:
			translationalSpeed -= incrementRate;
			ROS_INFO("New translational velocity = %f", translationalSpeed);
			break;
		case KEYCODE_I:
			rotationalSpeed += incrementRate;
			ROS_INFO("New rotational velocity = %f", rotationalSpeed);
			break;
		case KEYCODE_K:
			rotationalSpeed -= incrementRate;
			ROS_INFO("New rotational velocity = %f", rotationalSpeed);
			break;

		default:
			stopExperiment();
		}


		if (dirty == true) {
			baseCommandPublisher.publish(baseCommand);
		}
	}
}

void KeyboardTeleoperation::startExperiment() {

	experimentNumber ++;
	startTimeStamp = ros::Time::now();
	ROS_INFO_STREAM("Experiment #" << experimentNumber << ": Commanded velocities x,y,theta: " << baseCommand.linear.x << " " << baseCommand.linear.y << " " << baseCommand.angular.z);
	ROS_INFO_STREAM("Experiment #" << experimentNumber << ": Start pose (x,y): "  << lastReceivedOdometry.pose.pose.position.x << " " << lastReceivedOdometry.pose.pose.position.y);
	startPose = lastReceivedOdometry.pose.pose;

	experimentIsRunning = true;
}

void KeyboardTeleoperation::stopExperiment() {
    ROS_INFO("Stop experiment\n");
    baseCommand.linear.x = 0;
	baseCommand.linear.y = 0;
	baseCommand.angular.z = 0;
	if (experimentIsRunning == true) {
		ros::Duration timeSinceLastCommand = ros::Time::now() - startTimeStamp; //[s]

		stopPose = lastReceivedOdometry.pose.pose;
		double traveledDistanceX = fabs(stopPose.position.x - startPose.position.x); //[m]
		double traveledDistanceY = fabs(stopPose.position.y - startPose.position.y); //[m]
		double traveledDistance = sqrt(traveledDistanceX*traveledDistanceX + traveledDistanceY*traveledDistanceY); //[m]Pythagoras...

		double meanSpeed = traveledDistance / timeSinceLastCommand.sec; //[m/s]

		ROS_INFO_STREAM("Experiment #" << experimentNumber << ": Traveled time = " << timeSinceLastCommand);
		ROS_INFO_STREAM("Experiment #" << experimentNumber << ": End pose (x,y): "  << lastReceivedOdometry.pose.pose.position.x << " " << lastReceivedOdometry.pose.pose.position.y);
		ROS_INFO_STREAM("Experiment #" << experimentNumber << ": Traveled distance = " << traveledDistance);
		ROS_INFO_STREAM("Experiment #" << experimentNumber << ": Mean speed [m/s] = " << meanSpeed);
	}
	experimentIsRunning = false;
}

void KeyboardTeleoperation::baseOdometryCallback(const nav_msgs::Odometry& odomertry) {
	lastReceivedOdometry = odomertry;
	if (experimentIsRunning == true) {
		stopPose = lastReceivedOdometry.pose.pose;
		double traveledDistanceX = fabs(stopPose.position.x - startPose.position.x); //[m]
		double traveledDistanceY = fabs(stopPose.position.y - startPose.position.y); //[m]
		double traveledDistance = sqrt(traveledDistanceX*traveledDistanceX + traveledDistanceY*traveledDistanceY); //[m]Pythagoras...

		if (traveledDistance >= maxTraveledDistance) {
			ROS_INFO("autostop");
			baseCommand.linear.x = 0;
			baseCommand.linear.y = 0;
			baseCommand.angular.z = 0;
			baseCommandPublisher.publish(baseCommand);
			stopExperiment();
		}
	}
}

/* EOF */

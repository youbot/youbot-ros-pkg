/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Alexey Zakharov
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

#include <vector>
#include <string>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <sensor_msgs/JointState.h>

using namespace std;

vector <string> armJointStateNames;     //arm joint names, published by state_publisher from urdf model
vector <double> currentArmJointValues;  //arm joint value
vector <string> gripperJointStateNames; //gripper joint names, published by state_publisher from urdf model
double currentGripperJointValue;        //gripper joint value

vector <string> armCommandNames;        //arm command names, published by the user
string gripperCommandName;              //gripper command name, published by the user

const double deltaThreshold = 0.2;      //minimum value for time_from_start parameter
const double publishRate = 1;          //rate at which command are published, Hz;
                                        //used as a parameter for update rate and speed calculatins

bool mutex = false;                     //flag to prevent two processes from accessing currentArmJointValues at the same time

ros::Publisher armJointTrajectoryPublisher; //publisher of the updated trajectory message

void onJointStates(const sensor_msgs::JointState& jointStates) {
    if (mutex)
        return;
	/* create mapping between joint names and positions  */
    currentArmJointValues.clear();
	std::map<string, double> jointNameToValueMapping;
	for (unsigned int i = 0; i < jointStates.name.size(); ++i) {
		jointNameToValueMapping.insert(make_pair(jointStates.name[i], jointStates.position[i]));
	}

	/* fetch current arm joint values */
	vector <string> jointStateNames;
	for (unsigned int i = 0; i < armJointStateNames.size(); ++i) {
		map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(armJointStateNames[i]);
		if (jointIterator != jointNameToValueMapping.end()) {
            currentArmJointValues.push_back(jointIterator->second);
		}
	}

    /* mapping gripper_finger_joint_l and gripper_finger_joint_r to a single value*/
    currentGripperJointValue = 0;
    for (unsigned int i = 0; i < gripperJointStateNames.size(); ++i) {
        map<string, double>::const_iterator gripperIterator = jointNameToValueMapping.find(gripperJointStateNames[i]);
        if (gripperIterator != jointNameToValueMapping.end()) {
            currentGripperJointValue += fabs(gripperIterator->second);
        }
    }

}

trajectory_msgs::JointTrajectory republishedArmCommand;
trajectory_msgs::JointTrajectoryPoint armDesiredState;

void calculations() {

}

void onArmCommand(const trajectory_msgs::JointTrajectory& armCommand)
{

	if (armCommand.points.size() < 1){
		ROS_ERROR("youBot driver received an invalid joint trajectory command. Number of trajectory points > 1 is not supported yet.");
		return;
	}

	/* create mapping between joint names and values  */

	armDesiredState = armCommand.points[0];
	std::map<string, double> jointNameToValueMapping;
	for (int i = 0; i < static_cast<int>(armCommand.joint_names.size()); ++i) {
		jointNameToValueMapping.insert(make_pair(armCommand.joint_names[i], armDesiredState.positions[i]));
	}

	/* calculating  JointTrajectory.point[...].time_from_start for the arm (required parameter for Gazebo)*/
	double maxDelta = deltaThreshold;
	armDesiredState.positions.resize(0);

	republishedArmCommand.joint_names.resize(0);

	mutex = true;
	for (unsigned int i = 0; i < armCommandNames.size(); ++i) {
		/* check what is in map */
		map<string, double>::const_iterator armCommandIterator = jointNameToValueMapping.find(armCommandNames[i]);
		if (armCommandIterator != jointNameToValueMapping.end()) {
		    republishedArmCommand.joint_names.push_back(armCommandIterator->first);
		    armDesiredState.positions.push_back(armCommandIterator->second);
		    //armDesiredState.velocities.push_back(armCommandIterator->second);
		    /* calculating delta, which is proportional to time_from_start */
		    double delta = fabs(armDesiredState.positions.back() - currentArmJointValues[i]);
		    ROS_INFO("Delta = %f\n", delta);
		    if (delta > maxDelta)
                maxDelta = delta;   /*taking the maximum delta*/
		}
	}

    /*  calculating time_from_start for the gripper and
        mapping gripper_joint to gripper_finger_joint_l and gripper_finger_joint_r*/
	double gripperDesiredPosition = 0;
	map<string, double>::const_iterator gripperIterator = jointNameToValueMapping.find(gripperCommandName);
    if (gripperIterator != jointNameToValueMapping.end()) {
        gripperDesiredPosition = gripperIterator->second;
        republishedArmCommand.joint_names.push_back(gripperJointStateNames[0]);
        //armDesiredState.velocities.push_back(gripperDesiredPosition / 2.0);
        armDesiredState.positions.push_back(gripperDesiredPosition / 2.0);
        republishedArmCommand.joint_names.push_back(gripperJointStateNames[1]);
        //armDesiredState.velocities.push_back(gripperDesiredPosition / 2.0);
        armDesiredState.positions.push_back(gripperDesiredPosition / 2.0);
        double delta = fabs(gripperDesiredPosition - currentGripperJointValue);
        ROS_INFO("Delta = %f\n", delta);
        if (delta > maxDelta)
            maxDelta = delta;   /*taking the maximum delta*/
    }
    mutex = false;

   // maxDelta=

    /* Setting up new message */
	republishedArmCommand.header.stamp = armCommand.header.stamp;
	republishedArmCommand.header.frame_id = armCommand.header.frame_id;
	republishedArmCommand.points.resize(1); // only one point so far
	republishedArmCommand.points[0] = armDesiredState;
	const double k = 1;//0.000000001; // Proportional coefficient
	republishedArmCommand.points[0].time_from_start = ros::Duration(k); //filling time_from_start required parameter for Gazebo
	armJointTrajectoryPublisher.publish(republishedArmCommand);
}

void init() {

    armJointStateNames.push_back("arm_joint_1");
	armJointStateNames.push_back("arm_joint_2");
	armJointStateNames.push_back("arm_joint_3");
	armJointStateNames.push_back("arm_joint_4");
	armJointStateNames.push_back("arm_joint_5");

	gripperJointStateNames.push_back("gripper_finger_joint_l");
	gripperJointStateNames.push_back("gripper_finger_joint_r");

	armCommandNames = armJointStateNames;
	gripperCommandName = "gripper_joint";

}

int main(int argc, char **argv)
{
	init();

	ros::init(argc, argv, "arm_trajectory_republisher");
	ros::NodeHandle n;

	armJointTrajectoryPublisher = n.advertise<trajectory_msgs::JointTrajectory>("gazebo_arm_controller/command", 1);
 	ros::Subscriber jointStatesSubsriber = n.subscribe("joint_states", 1, onJointStates); // subscribing for joint states
 	ros::Subscriber armCommandsSubscriber = n.subscribe("arm_controller/command", 1, onArmCommand); // subscribing for joint states

	ros::Rate rate(publishRate); //Hz
	ros::AsyncSpinner spinner(4); // Use 4 threads
   	spinner.start();
    //ros::waitForShutdown();
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
      //  armJointTrajectoryPublisher.publish(republishedArmCommand);
       // ROS_INFO("Publishing\n");
    }
    return 0;
}

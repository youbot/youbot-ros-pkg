/******************************************************************************
 * Copyright (c) 2011
 * Locomotec, University of Twente
 *
 * Author:
 * Alexey Zakharov, Yury Brodskiy
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

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"


using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_arm_test");
	ros::NodeHandle n;
	ros::Publisher armJointTrajectoryPublisher;

	armJointTrajectoryPublisher = n.advertise<trajectory_msgs::JointTrajectory > ("arm_controller/position_command", 1);

	ros::Rate rate(20); //Hz
	double readValue;
	static const int numberOfJoints = 5;
	while (n.ok()) {

		trajectory_msgs::JointTrajectory armCommand;
		trajectory_msgs::JointTrajectoryPoint desiredConfiguration;

		desiredConfiguration.positions.resize(numberOfJoints + 1);  //5 arm joints + 1 gripper joint
		armCommand.joint_names.resize(numberOfJoints + 1);


		std::stringstream jointName;
		for (int i = 0; i < numberOfJoints; ++i) {
			cout << "Please type in value for joint " << i + 1 << endl;
			cin >> readValue;

			jointName.str("");
			jointName << "arm_joint_" << (i + 1);

			desiredConfiguration.positions[i] = readValue;
			armCommand.joint_names[i] = jointName.str();

		};

		cout << "Please type in value for gripper " << endl;
		cin >> readValue;
		desiredConfiguration.positions[numberOfJoints] = readValue;
		armCommand.joint_names[numberOfJoints] = "gripper_joint";


		armCommand.header.stamp = ros::Time::now();
		armCommand.header.frame_id = "base_link";
		armCommand.points.resize(1); // only one point so far
		armCommand.points[0] = desiredConfiguration;
        	armCommand.points[0].time_from_start = ros::Duration(0.0000001); // 1 ns

		cout << "sending command ..." << endl;
		armJointTrajectoryPublisher.publish(armCommand);
		cout << "--------------------" << endl;
		rate.sleep();
	}

	return 0;
}

/* EOF */

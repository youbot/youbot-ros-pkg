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

#include <boost/units/systems/si/length.hpp>
#include <boost/units/io.hpp>

#include <ros/ros.h>
#include <brics_actuator/CartesianPose.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace boost::units;

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_arm_cartesian_interaction_controller_test");
	ros::NodeHandle n;
	ros::Publisher armCommandPublisher;

	armCommandPublisher = n.advertise<brics_actuator::CartesianPose> ("arm_controller/command", 1);

	ros::Rate rate(20); //Hz
	while (n.ok()) {

		brics_actuator::CartesianVector tipPosition;
		geometry_msgs::Quaternion tipOrientation;
		float rollPithYaw[3];
		float xyz[3];

		cout << "Please type in end effector position (X Y Z) in meters: " << endl;
		cin >> tipPosition.x >> tipPosition.y >> tipPosition.z;
		tipPosition.unit = to_string(boost::units::si::meters);

		cout << "Please type in end effector orientation (Roll Pitch Yaw) in radians: " << endl;
		cin >> rollPithYaw[0] >> rollPithYaw[1] >> rollPithYaw[2];
		tipOrientation = tf::createQuaternionMsgFromRollPitchYaw(rollPithYaw[0], rollPithYaw[1], rollPithYaw[2]);

        brics_actuator::CartesianPose tipPose;
        tipPose.base_frame_uri = "/base_link";
        tipPose.target_frame_uri = "/arm_joint_5";
        tipPose.timeStamp = ros::Time::now();
        tipPose.position = tipPosition;
        tipPose.orientation = tipOrientation;

		cout << "sending command ..." << endl;
		armCommandPublisher.publish(tipPose);
		cout << "--------------------" << endl;
		rate.sleep();

	}

	return 0;
}

/* EOF */

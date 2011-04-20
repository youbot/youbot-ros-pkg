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

#include <boost/thread.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/io.hpp>

#include <ros/ros.h>
#include <brics_actuator/CartesianPose.h>
#include <tf/transform_datatypes.h>

#include <tf/transform_broadcaster.h>

using namespace std;
using namespace boost::units;

tf::TransformBroadcaster* br;
brics_actuator::CartesianPose tipPose;
bool mutex = true;

void publishTf() {

    while(ros::ok()) {
        if (mutex) {
            tf::Vector3 position(tipPose.position.x, tipPose.position.y, tipPose.position.z);
            tf::Quaternion orientation(tipPose.orientation.w, tipPose.orientation.x, tipPose.orientation.y, tipPose.orientation.z);
            tf::Transform transform(orientation, position);
            string parentFrameId = tipPose.base_frame_uri;
            string childFrameId = tipPose.target_frame_uri;
            br->sendTransform(tf::StampedTransform(transform,ros::Time::now(),parentFrameId,childFrameId));
            sleep(1);
        }
    }
}

void ypr2Quat(double yaw, double pitch, double roll, btQuaternion& quanternion) {
        btMatrix3x3 rotMatrix;
        rotMatrix.setEulerYPR(btScalar(yaw), btScalar(pitch), btScalar(roll));
        rotMatrix.getRotation(quanternion);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_arm_cartesian_interaction_controller_test");
	br = new tf::TransformBroadcaster();
	ros::NodeHandle n;
	ros::Publisher armCommandPublisher;

	armCommandPublisher = n.advertise<brics_actuator::CartesianPose> ("arm_controller/command", 1);

	ros::Rate rate(20); //Hz

	boost::thread thrd(&publishTf);


	while (ros::ok()) {

		brics_actuator::CartesianVector tipPosition;
		btQuaternion tipOrientation;
		float yawPithRoll[3];

		cout << "Please type in end effector position (X Y Z) in meters: " << endl;
		cin >> tipPosition.x >> tipPosition.y >> tipPosition.z;

		tipPosition.unit = to_string(boost::units::si::meters);

		cout << "Please type in end effector orientation (Yaw Pitch Roll) in radians: " << endl;
		cin >> yawPithRoll[0] >> yawPithRoll[1] >> yawPithRoll[2];

		ypr2Quat(yawPithRoll[0], yawPithRoll[1], yawPithRoll[2], tipOrientation);

        mutex = false;
        tipPose.base_frame_uri = "/arm_link_0";
        tipPose.target_frame_uri = "/target";
        tipPose.timeStamp = ros::Time::now();
        tipPose.position = tipPosition;
        tf::quaternionTFToMsg(tipOrientation, tipPose.orientation);
        mutex = true;
        cout << "sending command ..." << endl;
		armCommandPublisher.publish(tipPose);

		cout << "--------------------" << endl;
		rate.sleep();

	}

    thrd.join();
    delete br;
	return 0;
}

/* EOF */

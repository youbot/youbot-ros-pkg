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
            tf::Quaternion orientation;
            tf::quaternionMsgToTF(tipPose.orientation, orientation);
            tf::Transform transform(orientation, position);
            string parentFrameId = tipPose.base_frame_uri;
            string childFrameId = tipPose.target_frame_uri;
            br->sendTransform(tf::StampedTransform(transform,ros::Time::now(),parentFrameId,childFrameId));
            usleep(10000);
        }
    }
}

void ypr2Quat(double yaw, double pitch, double roll, btQuaternion& quanternion) {
        btMatrix3x3 rotMatrix;
        rotMatrix.setEulerYPR(btScalar(yaw), btScalar(pitch), btScalar(roll));
        rotMatrix.getRotation(quanternion);
}

void setCartesianVectorMsg(brics_actuator::CartesianPose& tipPose,
                           const brics_actuator::CartesianVector& tipPosition,
                           const btQuaternion& tipOrientation,
                           const string baseFrameUri,
                           const string targetFrameUri,
                           ros::Time timeStamp) {

    tipPose.base_frame_uri = baseFrameUri;
    tipPose.target_frame_uri = targetFrameUri;
    tipPose.timeStamp = timeStamp;
    tipPose.position = tipPosition;
    tf::quaternionTFToMsg(tipOrientation, tipPose.orientation);
}

// test to reach refference poses
const int delay = 10;
void test1(const ros::Publisher& armCommandPublisher) {

    for (int i = 0; i < 0; i++) {
    brics_actuator::CartesianVector tipPosition;
    btQuaternion tipOrientation;

    //ref. position 1
    tipPosition.x = 0.0;
    tipPosition.y = 0.0;
    tipPosition.z = 0.43;
    ypr2Quat(-1.0, 0.0, 0.0, tipOrientation);
    mutex = false;
    setCartesianVectorMsg(tipPose, tipPosition, tipOrientation, "/odom", "/target", ros::Time::now());
    mutex = true;
    sleep(2);
    cout << "sending command ..." << endl;
    armCommandPublisher.publish(tipPose);

    sleep(delay);
    //ref. position 2
    tipPosition.x = -0.1;
    tipPosition.y = -0.1;
    tipPosition.z = 0.4;
    ypr2Quat(-2.0, 1.3, 0.0, tipOrientation);
    mutex = false;
    setCartesianVectorMsg(tipPose, tipPosition, tipOrientation, "/odom", "/target", ros::Time::now());
    mutex = true;
    sleep(2);
    cout << "sending command ..." << endl;
    armCommandPublisher.publish(tipPose);

    sleep(delay);
    //ref. position 3
    tipPosition.x = 0.0;
    tipPosition.y = 0.2;
    tipPosition.z = 0.35;
    ypr2Quat(-3.0, 0.0, 1.0, tipOrientation);
    mutex = false;
    setCartesianVectorMsg(tipPose, tipPosition, tipOrientation, "/odom", "/target", ros::Time::now());
    mutex = true;
    sleep(2);
    cout << "sending command ..." << endl;
    armCommandPublisher.publish(tipPose);

    sleep(delay);
    //ref. position 4
    tipPosition.x = 0.2;
    tipPosition.y = 0.2;
    tipPosition.z = 0.35;
    ypr2Quat(-4.0, 1.0, 1.0, tipOrientation);
    mutex = false;
    setCartesianVectorMsg(tipPose, tipPosition, tipOrientation, "/odom", "/target", ros::Time::now());
    mutex = true;
    sleep(2);
    cout << "sending command ..." << endl;
    armCommandPublisher.publish(tipPose);

    sleep(delay);
    //ref. position 5
    tipPosition.x = 0.0;
    tipPosition.y = 0.2;
    tipPosition.z = 0.35;
    ypr2Quat(0.0, 0.0, 0.0, tipOrientation);
    mutex = false;
    setCartesianVectorMsg(tipPose, tipPosition, tipOrientation, "/odom", "/target", ros::Time::now());
    mutex = true;
    sleep(2);
    cout << "sending command ..." << endl;
    armCommandPublisher.publish(tipPose);

    sleep(delay);
    //ref. position 6
    tipPosition.x = 0.0;
    tipPosition.y = 0.2;
    tipPosition.z = 0.35;
    ypr2Quat(1.5, 0.0, 0.0, tipOrientation);
    mutex = false;
    setCartesianVectorMsg(tipPose, tipPosition, tipOrientation, "/odom", "/target", ros::Time::now());
    mutex = true;
    sleep(2);
    cout << "sending command ..." << endl;
    armCommandPublisher.publish(tipPose);
    sleep(delay);
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_arm_cartesian_interaction_controller_test");
    br = new tf::TransformBroadcaster();
	ros::NodeHandle n;
    ros::Publisher armCommandPublisher;

    brics_actuator::CartesianVector tipPosition;
    btQuaternion tipOrientation;

    tipPosition.x = 0;
    tipPosition.y = 0;
    tipPosition.z = 0;
    ypr2Quat(0.0, 0.0, 0.0, tipOrientation);
    setCartesianVectorMsg(tipPose, tipPosition, tipOrientation, "/odom", "/target", ros::Time::now());

	armCommandPublisher = n.advertise<brics_actuator::CartesianPose> ("gazebo_arm_controller/command", 1);


	ros::Rate rate(20); //Hz
	boost::thread thrd(&publishTf);

    test1(armCommandPublisher);

	while (ros::ok()) {


		float yawPithRoll[3];

		cout << "Please type in end effector position (X Y Z) in meters: " << endl;
		cin >> tipPosition.x >> tipPosition.y >> tipPosition.z;

		tipPosition.unit = to_string(boost::units::si::meters);

		cout << "Please type in end effector orientation (Yaw Pitch Roll) in radians: " << endl;
		cin >> yawPithRoll[0] >> yawPithRoll[1] >> yawPithRoll[2];

		ypr2Quat(yawPithRoll[0], yawPithRoll[1], yawPithRoll[2], tipOrientation);

        mutex = false;
        setCartesianVectorMsg(tipPose, tipPosition, tipOrientation, "/odom", "/target", ros::Time::now());
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

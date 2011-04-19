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
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace boost::units;

tf::TransformBroadcaster* br;
geometry_msgs::PoseStamped tipPose;
bool mutex = true;

void publishTf() {

    while(ros::ok()) {
        if (mutex) {
            tf::Vector3 position(tipPose.pose.position.x, tipPose.pose.position.y, tipPose.pose.position.z);
            tf::Quaternion orientation(tipPose.pose.orientation.w, tipPose.pose.orientation.x, tipPose.pose.orientation.y, tipPose.pose.orientation.z);
            tf::Transform transform(orientation, position);
            string parentFrameId = "/arm_link_0";
            string childFrameId = "/target";
            br->sendTransform(tf::StampedTransform(transform,ros::Time::now(),parentFrameId,childFrameId));
            sleep(1);
        }
    }
}

void rpy2Quat(double yaw, double pitch, double roll, geometry_msgs::Quaternion& quanternion) {
    // Assuming the angles are in radians.
    double c1 = cos(yaw);
    double s1 = sin(yaw);
    double c2 = cos(pitch);
    double s2 = sin(pitch);
    double c3 = cos(roll);
    double s3 = sin(roll);
    quanternion.w = sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;
    double w4 = (4.0 * quanternion.w);
    quanternion.x = (c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4 ;
    quanternion.y = (s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4 ;
    quanternion.z = (-s1 * s3 + c1 * s2 * c3 +s2) / w4 ;
  }

int main(int argc, char **argv) {

	ros::init(argc, argv, "youbot_arm_cartesian_pose_controller_test");
	br = new tf::TransformBroadcaster();
	ros::NodeHandle n;
	ros::Publisher armCommandPublisher;

	armCommandPublisher = n.advertise<geometry_msgs::PoseStamped> ("arm_controller/command", 1);

	ros::Rate rate(20); //Hz

	boost::thread thrd(&publishTf);


	while (ros::ok()) {

		geometry_msgs::Point tipPosition;
		geometry_msgs::Quaternion tipOrientation;
		float rollPithYaw[3];
		float xyz[3];

		cout << "Please type in end effector position (X Y Z) in meters: " << endl;
		cin >> tipPosition.x >> tipPosition.y >> tipPosition.z;


		cout << "Please type in end effector orientation (Roll Pitch Yaw) in radians: " << endl;
		cin >> rollPithYaw[0] >> rollPithYaw[1] >> rollPithYaw[2];

		rpy2Quat(rollPithYaw[0], rollPithYaw[1], rollPithYaw[2], tipOrientation);

        mutex = false;
        tipPose.header.frame_id = "/arm_link_5";

        tipPose.header.stamp = ros::Time::now();
        tipPose.pose.position = tipPosition;
        tipPose.pose.orientation = tipOrientation;
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

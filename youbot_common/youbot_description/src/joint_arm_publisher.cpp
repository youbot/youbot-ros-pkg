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

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "intial_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("youbot_arm_state", 1);
    ros::Rate loop_rate(10);

    double wheel = 0;

    // message declarations
    sensor_msgs::JointState joint_state;

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(5);
        joint_state.position.resize(5);
               
	// arm joint 1
	joint_state.name[0] ="arm_joint_1";
        joint_state.position[0] = 0;

	// arm joint 2
	joint_state.name[1] ="arm_joint_2";
        joint_state.position[1] = 0;

	// arm joint 3
	joint_state.name[2] ="arm_joint_3";
        joint_state.position[2] = 0;

	// arm joint 4
	joint_state.name[3] ="arm_joint_4";
        joint_state.position[2] = 0;

	// arm joint 5
	joint_state.name[4] ="arm_joint_5";
        joint_state.position[2] = 0;
	
        //publish the joint states
        joint_pub.publish(joint_state);

        wheel += 0.05;

	loop_rate.sleep();
    }

    return 0;
}

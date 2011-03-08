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
#include <geometry_msgs/Twist.h>

using namespace std;

ros::Publisher updatedBaseCommandPublisher; //publisher of the updated trajectory message
geometry_msgs::Twist updatedBaseCommand;

void onBaseCommand(const geometry_msgs::Twist currentBaseCommand) {
    updatedBaseCommand = currentBaseCommand;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_command_velocity_republisher");
	ros::NodeHandle n;
	updatedBaseCommandPublisher = n.advertise<geometry_msgs::Twist>("base_controller/command", 1);
 	ros::Subscriber currentBaseCommandSubsriber = n.subscribe("cmd_vel", 1, onBaseCommand); // subscribing for command velocity
 	ros::Rate r(10); // 10 hz
    while (true) {
        ros::spinOnce();
        updatedBaseCommandPublisher.publish(updatedBaseCommand);
        r.sleep();
    }
	return 0;
}

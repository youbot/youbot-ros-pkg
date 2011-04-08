/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Sebastian Blumenthal
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
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/systems/si/torque.hpp>
#include <boost/units/systems/si/force.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "youbot_arm_test");
	ros::NodeHandle n;
	ros::Publisher armJointVelocitiesPublisher;

	armJointVelocitiesPublisher = n.advertise<brics_actuator::CartesianWrench>("arm_controller/command", 1);

	ros::Rate rate(20); //Hz
	double readValue;
	static const int numberOfJoints = 5;
	while (n.ok()){
        brics_actuator::CartesianWrench command;
		brics_actuator::CartesianVector force;
        brics_actuator::CartesianVector torque;

		cout << "Please type in force values in " << boost::units::si::newton << endl;
		cin >> force.x >> force.y >> force.z;
		cout << "Please type in torque values in " <<  boost::units::si::newton_meter <<endl;
		cin >> torque.x >> torque.y >> torque.z;

        torque.unit = to_string(boost::units::si::newton_meter);
        force.unit = to_string(boost::units::si::newton);

		command.torque = torque;
		command.force = force;

		armJointVelocitiesPublisher.publish(command);
		cout << "--------------------" << endl;
		rate.sleep();

	}

  return 0;
}

/* EOF */

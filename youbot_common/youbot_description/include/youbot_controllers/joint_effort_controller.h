/******************************************************************************
 * Copyright (c) 2011
 * Locomotec
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

#ifndef JOINT_EFFORT_CONTROLLER_H
#define JOINT_EFFORT_CONTROLLER_H

#include <vector>

#include <ros/node_handle.h>
#include <control_toolbox/pid.h>
#include <pr2_controller_interface/controller.h>

#include "brics_actuator/JointTorques.h"
#include "brics_actuator/CartesianWrench.h"
#include "../src/force_control_generated/ForceControl.h"

namespace controller {

class JointEffortController : public pr2_controller_interface::Controller {
public:

	JointEffortController();
	~JointEffortController();

	void init20SimController();

	ForceControl my20simSubmodel;


	bool init(pr2_mechanism_model::RobotState *robotPtr, const std::string &jointName, const control_toolbox::Pid &pid);
	bool init(pr2_mechanism_model::RobotState *robotPtr, ros::NodeHandle &nodeHandle);
	void starting();
	void update();
	void udpate20SimControl(brics_actuator::CartesianWrench &wrench);

private:
	ros::NodeHandle nodeHandle;
	pr2_mechanism_model::RobotState *robotPtr;
	std::vector<pr2_mechanism_model::JointState*> joints;
	std::vector<control_toolbox::Pid> pids;
	ros::Time lastTime;
	std::vector <double> targetEfforts;
	brics_actuator::CartesianWrench wrench;
	XXDouble u [34 + 1];
	XXDouble y [27 + 1];
	ros::Subscriber subscriber;
	void velocityCommand(const brics_actuator::JointTorques &jointVelocities);
	void wrenchCommand(const brics_actuator::CartesianWrench &wrench);
};

} // namespace

#endif

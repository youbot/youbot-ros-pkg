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


#include <boost/units/systems/si.hpp>
#include <boost/units/systems/si/torque.hpp>
#include <boost/units/io.hpp>
#include "youbot_controllers/joint_effort_controller.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(youbot_description, JointEffortController, controller::JointEffortController, pr2_controller_interface::Controller)

namespace controller {

JointEffortController::JointEffortController()
: robotPtr(NULL) {

}

JointEffortController::~JointEffortController() {
	subscriber.shutdown();
}

/* when a controller gets initialized, the controller manager passes the controller a pointer to the RobotState
   RobotState is an interface to the robot joints and a description of the robot model
   for details see http://www.ros.org/wiki/pr2_mechanism_model */

bool JointEffortController::init(pr2_mechanism_model::RobotState *robotPtr, ros::NodeHandle &nodeHandle) {
	using namespace XmlRpc;
	this->nodeHandle = nodeHandle;
	this->robotPtr = robotPtr;

	ROS_INFO("Initializing effort control for the youbot arm...\n");

	// Gets all of the joint pointers from the RobotState to a joints vector
	XmlRpc::XmlRpcValue jointNames;
	if (!nodeHandle.getParam("joints", jointNames)) {
		ROS_ERROR("No joints given. (namespace: %s)", nodeHandle.getNamespace().c_str());
		return false;
	}

	if (jointNames.getType() != XmlRpc::XmlRpcValue::TypeArray) {
		ROS_ERROR("Malformed joint specification.  (namespace: %s)", nodeHandle.getNamespace().c_str());
		return false;
	}

	for (unsigned int i = 0; i < static_cast <unsigned int> (jointNames.size()); ++i) {
		XmlRpcValue &name = jointNames[i];
		if (name.getType() != XmlRpcValue::TypeString) {
			ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)", nodeHandle.getNamespace().c_str());
			return false;
		}

		pr2_mechanism_model::JointState *jointStatePtr = robotPtr->getJointState((std::string)name);
		if (jointStatePtr == NULL) {
			ROS_ERROR("Joint not found: %s. (namespace: %s)", ((std::string)name).c_str(), nodeHandle.getNamespace().c_str());
			return false;
		}

		joints.push_back(jointStatePtr);
	}

	// Ensures that all the joints are calibrated.
	for (unsigned int i = 0; i < joints.size(); ++i) {
		if (!joints[i]->calibrated_) {
			ROS_ERROR("Joint %s was not calibrated (namespace: %s)", joints[i]->joint_->name.c_str(), nodeHandle.getNamespace().c_str());
			return false;
		}
	}

	// Initializing targetefforts vector
	targetEfforts.resize(joints.size());

	// Sets up pid controllers for all of the joints from yaml file
	std::string gainsNS;

	if (!nodeHandle.getParam("gains", gainsNS))
		gainsNS = nodeHandle.getNamespace() + "/gains";

	pids.resize(joints.size());

	for (unsigned int i = 0; i < joints.size(); ++i) {
		if (!pids[i].init(ros::NodeHandle(gainsNS + "/" + joints[i]->joint_->name))) {
            ROS_ERROR("Can't setup PID for the joint %s. (namespace: %s)", joints[i]->joint_->name.c_str(), nodeHandle.getNamespace().c_str());
			return false;
		}

		double p,i,d,i_max,i_min;
		pids[i].getGains(p,i,d,i_max,i_min);
		ROS_DEBUG ("PID for joint %s: p=%f, i=%f, d=%f, i_max=%f, i_min=%f\n", joints[i]->joint_->name.c_str(), p,i,d,i_max,i_min);
	}

	subscriber = nodeHandle.subscribe("command", 1, &JointEffortController::velocityCommand, this);

	return true;
}

void JointEffortController::starting() {
	ROS_DEBUG("Starting velocity controls for the joints\n");

	for (unsigned int i = 0; i < pids.size(); ++i)
		pids[i].reset();

	// Initializing timer
	lastTime = robotPtr->getTime();
}

void JointEffortController::update() {
	// Calculating time interval dt between cycles
	ros::Time currentTime = robotPtr->getTime();
	ros::Duration dt = currentTime - lastTime;
	lastTime = currentTime;

	// Initializing error vector
	std::vector<double> error(joints.size());

	// Doing control here, calculating and applying the efforts
	for (unsigned int i = 0; i < joints.size(); ++i) {
		joints[i]->commanded_effort_ = targetEfforts[i];
    }
    ROS_INFO("%f\n", joints[1]->commanded_effort_);
}

void JointEffortController::velocityCommand(const brics_actuator::JointTorques &jointEfforts) {
	ROS_DEBUG("Readin the target velocity from brics_actuator::jointEfforts message\n");
	std::vector <brics_actuator::JointValue> efforts = jointEfforts.torques;

	if (efforts.empty()) {
		starting();
		return;
	}

	//Correlates the joints we're commanding to the joints in the message
	std::vector<int> lookup(joints.size(), -1); // Maps from an index in joints_ to an index in the msg
	for (unsigned int j = 0; j < joints.size(); ++j) {

		for (unsigned int k = 0; k < efforts.size(); ++k) {
			if (efforts[k].joint_uri == joints[j]->joint_->name) {
				lookup[j] = k;
				break;
			}
		}
		if (lookup[j] == -1)
			ROS_ERROR("Unable to locate joint %s in the commanded efforts.", joints[j]->joint_->name.c_str());
	}

	std::vector<double> actualefforts;
	targetEfforts.resize(efforts.size());
	using namespace boost::units;

    for (unsigned int j = 0; j < joints.size(); ++j) {
        if (lookup[j] != -1) {
            ROS_DEBUG("Joint %s = %f %s, ",efforts[lookup[j]].joint_uri.c_str(), efforts[lookup[j]].value, efforts[lookup[j]].unit.c_str());
            if (efforts[lookup[j]].unit != to_string(si::newton_meter))
                ROS_ERROR("Joint %s has the value in the inpcompatible units %s", efforts[lookup[j]].joint_uri.c_str(), efforts[lookup[j]].unit.c_str());
            if (!targetEfforts.empty())
                targetEfforts[j] = efforts[lookup[j]].value;
        }
	}
}
}

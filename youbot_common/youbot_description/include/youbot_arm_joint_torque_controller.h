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
#include <boost/scoped_ptr.hpp>
#include <ros/node_handle.h>
#include <control_toolbox/pid.h>
#include <pr2_controller_interface/controller.h>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>

#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"

#include "brics_actuator/JointTorques.h"
#include "brics_actuator/CartesianWrench.h"
#include "brics_actuator/CartesianPose.h"
#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointTorques.h"

#include "20_sim_interaction_control/position_interaction_control.h"

namespace controller {

class CartesianInteractionController : public pr2_controller_interface::Controller {
public:

	CartesianInteractionController();
	~CartesianInteractionController();

	void init20SimController();
	bool init(pr2_mechanism_model::RobotState *robotPtr, const std::string &jointName, const control_toolbox::Pid &pid);
	bool init(pr2_mechanism_model::RobotState *robotPtr, ros::NodeHandle &nodeHandle);
	void starting();
	void update();
	void publish();
	void udpate20SimControl(brics_actuator::CartesianWrench &wrench);

private:
    PositionInteractionControl 20simController; /* auto-generated 20 sim controller    */
	XXDouble u [34 + 1]; /* input parameters for the auto-generated 20 sim controller  */
	XXDouble y [21 + 1]; /* output parameters for the auto-generated 20 sim controller */

	ros::Subscriber subscriber;
	ros::NodeHandle nodeHandle;
	pr2_mechanism_model::RobotState *robotPtr;
	std::vector<pr2_mechanism_model::JointState*> joints;

	ros::Time lastTime;
	unsigned int loopCount;
	double orientation[3];
    double position[3];
    std::vector <double> targetEfforts;

	void positionCommand(const brics_actuator::JointTorques &jointVelocities);

//#ifdef DEBUG
	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::JointPositions> > gazeboJointPositions;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::JointPositions> > controllerJointPositions;

	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::JointTorques> > gazeboJointTorques;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::JointTorques> > controllerJointTorques;

	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::CartesianPose> > gazeboJointPose;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<brics_actuator::CartesianPose> > controllerJointPose;
//#endif
};

} // namespace

#endif

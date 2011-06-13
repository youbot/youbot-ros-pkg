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

#ifndef YOUBOTCONFIGURATION_H_
#define YOUBOTCONFIGURATION_H_

#include "ros/ros.h"

#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointVelocities.h"

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

namespace youBot {

class YouBotBaseConfiguration {
public:
	YouBotBaseConfiguration();
	virtual ~YouBotBaseConfiguration();

	/// "Name" of the base. Typically derived from name of youBot configuration file.
	std::string baseID;

	/// Handle to the base
	youbot::YouBotBase* youBotBase;

	ros::Subscriber baseCommandSubscriber;


};

class YouBotArmConfiguration {
public:
	YouBotArmConfiguration();
	virtual ~YouBotArmConfiguration();

	std::string armID;

	/// Handle to the arm
	youbot::YouBotManipulator* youBotArm;

	std::string commandTopicName;
	std::string parentFrameIDName;
	std::map<std::string, int> jointNameToJointIndexMapping;

	ros::Subscriber armPositionCommandSubscriber;
	ros::Subscriber armVelocityCommandSubscriber;
	ros::Subscriber gripperPositionCommandSubscriber;

};

/**
 * @brief Aggregation class for instantiated parts of a youBot systems and all its name mapping between ROS and the youBot API.
 */
class YouBotConfiguration {
public:
	YouBotConfiguration();
	virtual ~YouBotConfiguration();

	std:: string configurationFilePath;
	bool hasBase;
	/// True for one ore more arms
	bool hasArms;

	YouBotBaseConfiguration baseConfiguration;
	std::vector<YouBotArmConfiguration> youBotArmConfigurations;
	std::map<std::string, int> armNameToArmIndexMapping;
};




}  // namespace youBot

#endif /* YOUBOTCONFIGURATION_H_ */

/* EOF */

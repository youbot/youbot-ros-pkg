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

#include "YouBotConfiguration.h"

namespace youBot {

YouBotBaseConfiguration::YouBotBaseConfiguration() {
	youBotBase = 0;

	/* provide some default values for the joint names (might be overwritten) */
	/*
	 *  numbering of youBot wheels:
	 *
	 *    FRONT
	 *
	 * 1 ---+--- 2
	 *      |
	 *      |
	 *      |
	 *      |
	 * 3 ---+--- 4
	 *
	 *    BACK
	 */
	wheelNames.clear();
	wheelNames.push_back("wheel_joint_fl"); //wheel #1
	wheelNames.push_back("wheel_joint_fr"); //wheel #2
	wheelNames.push_back("wheel_joint_bl"); //wheel #3
	wheelNames.push_back("wheel_joint_br"); //wheel #4
}

YouBotBaseConfiguration::~YouBotBaseConfiguration() {
	if (youBotBase) {
		delete youBotBase;
		youBotBase = 0;
	}


}

YouBotArmConfiguration::YouBotArmConfiguration() {
	youBotArm = 0;

	/* provide some default values for the joint names (might be overwritten) */
	jointNames.clear();
	jointNames.push_back("arm_joint_1");
	jointNames.push_back("arm_joint_2");
	jointNames.push_back("arm_joint_3");
	jointNames.push_back("arm_joint_4");
	jointNames.push_back("arm_joint_5");

	gripperFingerNames.clear();
	gripperFingerNames.push_back("gripper_finger_joint_l");
	gripperFingerNames.push_back("gripper_finger_joint_r");
}

YouBotArmConfiguration::~YouBotArmConfiguration() {
	if (youBotArm) {
		delete youBotArm;
		youBotArm = 0;
	}
	jointNames.clear();
}

YouBotConfiguration::YouBotConfiguration() {
	youBotArmConfigurations.clear();
	armNameToArmIndexMapping.clear();
	hasBase = false;
	hasArms = false;

}

YouBotConfiguration::~YouBotConfiguration() {
	youBotArmConfigurations.clear();
	armNameToArmIndexMapping.clear();
}

}  // namespace youBot

/* EOF */

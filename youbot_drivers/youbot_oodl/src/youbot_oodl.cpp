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

#include "YouBotOODLWrapper.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "youbot_oodl_driver");
	ros::NodeHandle n;
	youBot::YouBotOODLWrapper youBot(n);


	/* configuration */
	bool youBotHasBase;
	bool youBotHasArm;
	n.param("youBotHasBase", youBotHasBase, true);
	n.param("youBotHasArm", youBotHasArm, true);
	n.param<std::string>("youBotConfigurationFilePath", youBot.youBotConfiguration.configurationFilePath, mkstr(YOUBOT_CONFIGURATIONS_DIR));

	ROS_ASSERT((youBotHasBase == true) || (youBotHasArm == true)); // At least one should be true, otherwise nothing to be started.
	if (youBotHasBase == true) {
		youBot.initializeBase("youbot-base");
	}

	if (youBotHasArm == true) {
//		youBot.initializeArm("youbot-manipulator");
		youBot.initializeArm("youbot-manipulator1");
//		youBot.initializeArm("youbot-manipulator2");
	}


	/* coordination */
	ros::Rate rate(20); //Input and output at the same time... (in Hz)
	while (n.ok()){
		ros::spinOnce();
		youBot.computeOODLSensorReadings();
		youBot.publishOODLSensorReadings();
		rate.sleep();
	}

	youBot.stop();

  return 0;
}


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
#include "joint_state_observer_oodl.h"

#include <youbot_trajectory_action_server/joint_trajectory_action.h>
#include <sstream>

namespace youBot
{

YouBotOODLWrapper::YouBotOODLWrapper()
{
}

YouBotOODLWrapper::YouBotOODLWrapper(ros::NodeHandle n) :
node(n)
{

    youBotConfiguration.hasBase = false;
    youBotConfiguration.hasArms = false;

    youBotChildFrameID = "base_link"; //holds true for both: base and arm
    armJointStateMessages.clear();

    n.param("youBotDriverCycleFrequencyInHz", youBotDriverCycleFrequencyInHz, 50.0);
    n.param("trajectoryActionServerEnable", trajectoryActionServerEnable, false);
    n.param("trajectoryVelocityGain", trajectoryVelocityGain, 0.0);
    n.param("trajectoryPositionGain", trajectoryPositionGain, 5.0);
	gripperCycleCounter = 0;
}

YouBotOODLWrapper::~YouBotOODLWrapper()
{
    this->stop();
}

void YouBotOODLWrapper::initializeBase(std::string baseName)
{

    try
    {
        ROS_INFO("Configuration file path: %s", youBotConfiguration.configurationFilePath.c_str());
        youBotConfiguration.baseConfiguration.youBotBase = new youbot::YouBotBase(baseName, youBotConfiguration.configurationFilePath);
        youBotConfiguration.baseConfiguration.youBotBase->doJointCommutation();
    }
    catch (std::exception& e)
    {
        std::string errorMessage = e.what();
        ROS_FATAL("Cannot open youBot driver: \n %s ", errorMessage.c_str());
        ROS_ERROR("Base \"%s\" could not be initialized.", baseName.c_str());
        youBotConfiguration.hasBase = false;
        return;
    }

    /* setup input/output communication */
    youBotConfiguration.baseConfiguration.baseCommandSubscriber = node.subscribe("cmd_vel", 1000, &YouBotOODLWrapper::baseCommandCallback, this);
    youBotConfiguration.baseConfiguration.baseOdometryPublisher = node.advertise<nav_msgs::Odometry > ("odom", 1);
    youBotConfiguration.baseConfiguration.baseJointStatePublisher = node.advertise<sensor_msgs::JointState > ("base/joint_states", 1);

    /* setup services*/
    youBotConfiguration.baseConfiguration.switchOffMotorsService = node.advertiseService("base/switchOffMotors", &YouBotOODLWrapper::switchOffBaseMotorsCallback, this);
    youBotConfiguration.baseConfiguration.switchONMotorsService = node.advertiseService("base/switchOnMotors", &YouBotOODLWrapper::switchOnBaseMotorsCallback, this);

    /* setup frame_ids */
    youBotOdometryFrameID = "odom";
    youBotOdometryChildFrameID = "base_footprint";

    ROS_INFO("Base is initialized.");
    youBotConfiguration.hasBase = true;
}

void YouBotOODLWrapper::initializeArm(std::string armName, bool enableStandardGripper)
{
    int armIndex;
    youbot::JointName jointNameParameter;
    std::string jointName;
    stringstream topicName;
    stringstream serviceName;

    try
    {
        ROS_INFO("Configuration file path: %s", youBotConfiguration.configurationFilePath.c_str());   
        YouBotArmConfiguration tmpArmConfig;
        youBotConfiguration.youBotArmConfigurations.push_back(tmpArmConfig);
        armIndex = static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()) - 1;
        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm = new youbot::YouBotManipulator(armName, youBotConfiguration.configurationFilePath);
        youBotConfiguration.youBotArmConfigurations[armIndex].armID = armName;
        topicName.str("");
        topicName << "arm_" << (armIndex + 1) << "/";
        youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName = topicName.str(); // e.g. arm_1/
        youBotConfiguration.youBotArmConfigurations[armIndex].parentFrameIDName = "base_link";
        youBotConfiguration.armNameToArmIndexMapping.insert(make_pair(armName, static_cast<int> (youBotConfiguration.youBotArmConfigurations.size())));

        /* take joint names form configuration files */
        youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.clear();
        for (int i = 0; i < youBotArmDoF; ++i)
        {
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getConfigurationParameter(jointNameParameter);
            jointNameParameter.getParameter(jointName);
            youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.push_back(jointName);
            ROS_INFO("Joint %i for arm %s has name: %s", i + 1, youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str(), youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i].c_str());

        }


        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->doJointCommutation();
        youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->calibrateManipulator();
        if (enableStandardGripper)
        {
        	youbot::GripperBarName barName;
        	std::string gripperBarName;

            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar1().getConfigurationParameter(barName);
            barName.getParameter(gripperBarName);
            youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX] = gripperBarName;
            ROS_INFO("Joint %i for gripper of arm %s has name: %s", 1, youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str(), gripperBarName.c_str());

            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar2().getConfigurationParameter(barName);
            barName.getParameter(gripperBarName);
            youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX] = gripperBarName;
            ROS_INFO("Joint %i for gripper of arm %s has name: %s", 2, youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str(), gripperBarName.c_str());

            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->calibrateGripper();
        }
    }
    catch (std::exception& e)
    {
        youBotConfiguration.youBotArmConfigurations.pop_back();
        std::string errorMessage = e.what();
        ROS_FATAL("Cannot open youBot driver: \n %s ", errorMessage.c_str());
        ROS_ERROR("Arm \"%s\" could not be initialized.", armName.c_str());
        ROS_INFO("System has %i initialized arm(s).", static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));
        return;
    }


    /* setup input/output communication */
    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/position_command"; // e.g. arm_1/arm_controller/positionCommand
    youBotConfiguration.youBotArmConfigurations[armIndex].armPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::armPositionsCommandCallback, this, _1, armIndex));

    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/velocity_command";
    youBotConfiguration.youBotArmConfigurations[armIndex].armVelocityCommandSubscriber = node.subscribe<brics_actuator::JointVelocities > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::armVelocitiesCommandCallback, this, _1, armIndex));

    topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "joint_states";
    youBotConfiguration.youBotArmConfigurations[armIndex].armJointStatePublisher = node.advertise<sensor_msgs::JointState > (topicName.str(), 1); //TODO different names or one topic?

    if (enableStandardGripper)
    {
        topicName.str("");
        topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "gripper_controller/position_command";
        youBotConfiguration.youBotArmConfigurations[armIndex].gripperPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::gripperPositionsCommandCallback, this, _1, armIndex));
        youBotConfiguration.youBotArmConfigurations[armIndex].lastGripperCommand = 0.0; //This is true if the gripper is calibrated.
    }



    /* setup services*/
    serviceName.str("");
    serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "switchOffMotors"; // e.g. "arm_1/switchOffMotors"
    youBotConfiguration.youBotArmConfigurations[armIndex].switchOffMotorsService = node.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response > (serviceName.str(), boost::bind(&YouBotOODLWrapper::switchOffArmMotorsCallback, this, _1, _2, armIndex));

    serviceName.str("");
    serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "switchOnMotors"; // e.g. "arm_1/switchOnMotors"
    youBotConfiguration.youBotArmConfigurations[armIndex].switchONMotorsService = node.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response > (serviceName.str(), boost::bind(&YouBotOODLWrapper::switchOnArmMotorsCallback, this, _1, _2, armIndex));

    serviceName.str("");
    serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "calibrate"; // e.g. "arm_1/calibrate"
    youBotConfiguration.youBotArmConfigurations[armIndex].calibrateService = node.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response > (serviceName.str(), boost::bind(&YouBotOODLWrapper::calibrateArmCallback, this, _1, _2, armIndex));

    if (trajectoryActionServerEnable)
    {
        JointStateObserver* jointStateObserver = new JointStateObserverOODL(this, armIndex);
        topicName.str("");
        topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "action";
        youBotConfiguration.youBotArmConfigurations[armIndex].jointTrajectoryAction = new JointTrajectoryAction(jointStateObserver,
                                                                                                                trajectoryPositionGain,
                                                                                                                trajectoryVelocityGain,
                                                                                                                youBotDriverCycleFrequencyInHz);

        serviceName.str("");
        serviceName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/joint_trajectory_action"; // e.g. "arm_1/switchOnMotors"
        
        youBotConfiguration.youBotArmConfigurations[armIndex].trajectoryActionServer = new Server(serviceName.str(),
                                                                                                  boost::bind(&YouBotOODLWrapper::executeActionServer, this, _1, armIndex),
                                                                                                  false);
        youBotConfiguration.youBotArmConfigurations[armIndex].trajectoryActionServer->start();
    }

    /* initialize message vector for arm joint states */
    sensor_msgs::JointState dummyMessage;
    armJointStateMessages.push_back(dummyMessage);

    /* setup frame_ids */
    youBotArmFrameID = "arm"; //TODO find default topic name
    ROS_INFO("Arm \"%s\" is initialized.", armName.c_str());
    ROS_INFO("System has %i initialized arm(s).", static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));
    youBotConfiguration.hasArms = true;
}

void YouBotOODLWrapper::executeActionServer(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, int armIndex)
{

    JointTrajectoryAction* jointTrajectoryAction = youBotConfiguration.youBotArmConfigurations[armIndex].jointTrajectoryAction;
    if (jointTrajectoryAction != NULL)
    {

        jointTrajectoryAction->execute(goal, youBotConfiguration.youBotArmConfigurations[armIndex].trajectoryActionServer);
    }
}

void YouBotOODLWrapper::stop()
{

    if (youBotConfiguration.hasBase)
    {
        if (youBotConfiguration.baseConfiguration.youBotBase)
        {
            delete youBotConfiguration.baseConfiguration.youBotBase;
            youBotConfiguration.baseConfiguration.youBotBase = 0;
        }

        youBotConfiguration.baseConfiguration.baseCommandSubscriber.shutdown();
        youBotConfiguration.baseConfiguration.baseJointStatePublisher.shutdown();
        youBotConfiguration.baseConfiguration.baseOdometryPublisher.shutdown();
        youBotConfiguration.baseConfiguration.switchONMotorsService.shutdown();
        youBotConfiguration.baseConfiguration.switchOffMotorsService.shutdown();
        // youBotConfiguration.baseConfiguration.odometryBroadcaster.
        youBotConfiguration.hasBase = false;
    }

    if (youBotConfiguration.hasArms)
    {
        for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++) //delete each arm
        {
            if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm)
            {
                delete youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm;
                youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm = 0;
            }

            youBotConfiguration.youBotArmConfigurations[armIndex].armJointStatePublisher.shutdown();
            youBotConfiguration.youBotArmConfigurations[armIndex].armPositionCommandSubscriber.shutdown();
            youBotConfiguration.youBotArmConfigurations[armIndex].armVelocityCommandSubscriber.shutdown();
            youBotConfiguration.youBotArmConfigurations[armIndex].calibrateService.shutdown();
            youBotConfiguration.youBotArmConfigurations[armIndex].gripperPositionCommandSubscriber.shutdown();
            youBotConfiguration.youBotArmConfigurations[armIndex].switchONMotorsService.shutdown();
            youBotConfiguration.youBotArmConfigurations[armIndex].switchOffMotorsService.shutdown();
        }

        youBotConfiguration.hasArms = false;
        youBotConfiguration.youBotArmConfigurations.clear();
        armJointStateMessages.clear();
    }
    youbot::EthercatMaster::destroy();
}

void YouBotOODLWrapper::baseCommandCallback(const geometry_msgs::Twist& youbotBaseCommand)
{

    if (youBotConfiguration.hasBase)
    { // in case stop has been invoked
        quantity<si::velocity> longitudinalVelocity;
        quantity<si::velocity> transversalVelocity;
        quantity<si::angular_velocity> angularVelocity;

        /*
         * Frame in OODL:
         *
         *		 FRONT
         *
         *         X
         *         ^
         *         |
         *         |
         *         |
         * Y <-----+
         *
         *        BACK
         *
         * Positive angular velocity means turning counterclockwise
         *
         */

        longitudinalVelocity = youbotBaseCommand.linear.x * meter_per_second;
        transversalVelocity = youbotBaseCommand.linear.y * meter_per_second;
        angularVelocity = youbotBaseCommand.angular.z * radian_per_second;

        try
        {
            youBotConfiguration.baseConfiguration.youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot set base velocities: \n %s", errorMessage.c_str());
        }

    }
    else
    {
        ROS_ERROR("No base initialized!");
    }
}

void YouBotOODLWrapper::armPositionsCommandCallback(const brics_actuator::JointPositionsConstPtr& youbotArmCommand, int armIndex)
{
    ROS_DEBUG("Command for arm%i received", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) // in case stop has been invoked
    {

        ROS_DEBUG("Arm ID is: %s", youBotConfiguration.youBotArmConfigurations[armIndex].armID.c_str());
        if (youbotArmCommand->positions.size() < 1)
        {
            ROS_WARN("youBot driver received an invalid joint positions command.");
            return;
        }

        youbot::JointAngleSetpoint desiredAngle;
        string unit = boost::units::to_string(boost::units::si::radian);

        /* populate mapping between joint names and values  */
        std::map<string, double> jointNameToValueMapping;
        for (int i = 0; i < static_cast<int> (youbotArmCommand->positions.size()); ++i)
        {
            if (unit == youbotArmCommand->positions[i].unit)
            {
                jointNameToValueMapping.insert(make_pair(youbotArmCommand->positions[i].joint_uri, youbotArmCommand->positions[i].value));
            }
            else
            {
                ROS_WARN("Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotArmCommand->positions[i].unit.c_str(), unit.c_str());
            }
        }

        /* loop over all youBot arm joints and check if something is in the received message that requires action */
        ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
        for (int i = 0; i < youBotArmDoF; ++i)
        {

            /* check what is in map */
            map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]);
            if (jointIterator != jointNameToValueMapping.end())
            {

                /* set the desired joint value */
                ROS_DEBUG("Trying to set joint %s to new position value %f", (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]).c_str(), jointIterator->second);
                desiredAngle.angle = jointIterator->second * radian;
                try
                {
                    youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).setData(desiredAngle); //youBot joints start with 1 not with 0 -> i + 1
                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: \n %s", i + 1, errorMessage.c_str());
                }
            }
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
    }
    else
    {
        ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
    }

}

void YouBotOODLWrapper::armVelocitiesCommandCallback(const brics_actuator::JointVelocitiesConstPtr& youbotArmCommand, int armIndex)
{
    ROS_DEBUG("Command for arm%i received", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0)
    { // in case stop has been invoked


        if (youbotArmCommand->velocities.size() < 1)
        {
            ROS_WARN("youBot driver received an invalid joint velocities command.");
            return;
        }

        youbot::JointVelocitySetpoint desiredAngularVelocity;
        string unit = boost::units::to_string(boost::units::si::radian_per_second);

        /* populate mapping between joint names and values  */
        std::map<string, double> jointNameToValueMapping;
        for (int i = 0; i < static_cast<int> (youbotArmCommand->velocities.size()); ++i)
        {
            if (unit == youbotArmCommand->velocities[i].unit)
            {
                jointNameToValueMapping.insert(make_pair(youbotArmCommand->velocities[i].joint_uri, youbotArmCommand->velocities[i].value));
            }
            else
            {
                ROS_WARN("Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotArmCommand->velocities[i].unit.c_str(), unit.c_str());
            }

        }

        /* loop over all youBot arm joints and check if something is in the received message that requires action */
        ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
        for (int i = 0; i < youBotArmDoF; ++i)
        {

            /* check what is in map */
            map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]);
            if (jointIterator != jointNameToValueMapping.end())
            {

                /* set the desired joint value */
                ROS_DEBUG("Trying to set joint %s to new velocity value %f", (youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]).c_str(), jointIterator->second);
                desiredAngularVelocity.angularVelocity = jointIterator->second * radian_per_second;
                try
                {
                    youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).setData(desiredAngularVelocity); //youBot joints start with 1 not with 0 -> i + 1

                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: \n %s", i + 1, errorMessage.c_str());
                }
            }
        }
        youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
    }
    else
    {
        ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
    }
}

void YouBotOODLWrapper::gripperPositionsCommandCallback(const brics_actuator::JointPositionsConstPtr& youbotGripperCommand, int armIndex)
{
	ROS_DEBUG("Command for gripper%i received", armIndex + 1);
	ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));

	if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) { // in case stop has been invoked

		if (youbotGripperCommand->positions.size() < 1){
			ROS_WARN("youBot driver received an invalid gripper positions command.");
			return;
		}

		map<string, double>::const_iterator gripperIterator;
		youbot::GripperBarPositionSetPoint leftGripperFingerPosition;
		youbot::GripperBarPositionSetPoint rightGripperFingerPosition;
		string unit = boost::units::to_string(boost::units::si::meter);

		/* populate mapping between joint names and values */
		std::map<string, double> jointNameToValueMapping;
		for (int i = 0; i < static_cast<int>(youbotGripperCommand->positions.size()); ++i) {
			if (unit == youbotGripperCommand->positions[i].unit) {
				jointNameToValueMapping.insert(make_pair(youbotGripperCommand->positions[i].joint_uri, youbotGripperCommand->positions[i].value));
			} else {
				ROS_WARN("Unit incompatibility. Are you sure you want to command %s instead of %s ?", youbotGripperCommand->positions[i].unit.c_str(), unit.c_str());
			}
		}

		youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time

		/* check if something is in the received message that requires action for the left finger gripper */
		gripperIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX]);
		if (gripperIterator != jointNameToValueMapping.end()) {
			ROS_DEBUG("Trying to set the left gripper finger to new value %f", gripperIterator->second);

			leftGripperFingerPosition.barPosition = gripperIterator->second * meter;
			try {
				youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar1().setData(leftGripperFingerPosition);
			} catch (std::exception& e) {
				std::string errorMessage = e.what();
				ROS_WARN("Cannot set the left gripper finger: \n %s", errorMessage.c_str());
			}
		}

		/* check if something is in the received message that requires action for the right finger gripper */
		gripperIterator = jointNameToValueMapping.find(youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX]);
		if (gripperIterator != jointNameToValueMapping.end()) {
			ROS_DEBUG("Trying to set the right gripper to new value %f", gripperIterator->second);

			rightGripperFingerPosition.barPosition = gripperIterator->second * meter;
			try {
				youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar2().setData(rightGripperFingerPosition);
			} catch (std::exception& e) {
				std::string errorMessage = e.what();
				ROS_WARN("Cannot set the right gripper finger: \n %s", errorMessage.c_str());
			}
		}

		youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
	} else {
		ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
	}
}

void YouBotOODLWrapper::computeOODLSensorReadings()
{

    currentTime = ros::Time::now();
    youbot::JointSensedAngle currentAngle;
    youbot::JointSensedVelocity currentVelocity;

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false); // ensure that all joint values will be send at the same time

    if (youBotConfiguration.hasBase == true)
    {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;

        double vx = 0.0;
        double vy = 0.0;
        double vtheta = 0.0;

        quantity<si::length> longitudinalPosition;
        quantity<si::length> transversalPosition;
        quantity<plane_angle> orientation;

        quantity<si::velocity> longitudinalVelocity;
        quantity<si::velocity> transversalVelocity;
        quantity<si::angular_velocity> angularVelocity;

        youBotConfiguration.baseConfiguration.youBotBase->getBasePosition(longitudinalPosition, transversalPosition, orientation);
        x = longitudinalPosition.value();
        y = transversalPosition.value();
        theta = orientation.value();

        youBotConfiguration.baseConfiguration.youBotBase->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        vx = longitudinalVelocity.value();
        vy = transversalVelocity.value();
        vtheta = angularVelocity.value();
        //ROS_DEBUG("Perceived odometric values (x,y,tetha, vx,vy,vtetha): %f, %f, %f \t %f, %f, %f", x, y, theta, vx, vy, vtheta);


        /* Setup odometry tf frame */
        odometryQuaternion = tf::createQuaternionMsgFromYaw(theta);

        odometryTransform.header.stamp = currentTime;
        odometryTransform.header.frame_id = youBotOdometryFrameID;
        odometryTransform.child_frame_id = youBotOdometryChildFrameID;

        odometryTransform.transform.translation.x = x;
        odometryTransform.transform.translation.y = y;
        odometryTransform.transform.translation.z = 0.0;
        odometryTransform.transform.rotation = odometryQuaternion;

        /* Setup odometry Message */
        odometryMessage.header.stamp = currentTime;
        odometryMessage.header.frame_id = youBotOdometryFrameID;

        odometryMessage.pose.pose.position.x = x;
        odometryMessage.pose.pose.position.y = y;
        odometryMessage.pose.pose.position.z = 0.0;
        odometryMessage.pose.pose.orientation = odometryQuaternion;

        odometryMessage.child_frame_id = youBotOdometryChildFrameID;
        //		odometryMessage.child_frame_id = youBotOdometryFrameID;
        odometryMessage.twist.twist.linear.x = vx;
        odometryMessage.twist.twist.linear.y = vy;
        odometryMessage.twist.twist.angular.z = vtheta;

        /* Set up joint state message for the wheels */
        baseJointStateMessage.header.stamp = currentTime;
        baseJointStateMessage.name.resize(youBotNumberOfWheels * 2); // *2 because of virtual wheel joints in the URDF description
        baseJointStateMessage.position.resize(youBotNumberOfWheels * 2);
        baseJointStateMessage.velocity.resize(youBotNumberOfWheels * 2);

        ROS_ASSERT((youBotConfiguration.baseConfiguration.wheelNames.size() == static_cast<unsigned int> (youBotNumberOfWheels)));
        for (int i = 0; i < youBotNumberOfWheels; ++i)
        {
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i + 1).getData(currentAngle); //youBot joints start with 1 not with 0 -> i + 1
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i + 1).getData(currentVelocity);

            baseJointStateMessage.name[i] = youBotConfiguration.baseConfiguration.wheelNames[i];
            baseJointStateMessage.position[i] = currentAngle.angle.value();
            baseJointStateMessage.velocity[i] = currentVelocity.angularVelocity.value();
        }

        /*
         * Here we add values for "virtual" rotation joints in URDF - robot_state_publisher can't
         * handle non-aggregated jointState messages well ...
         */
        baseJointStateMessage.name[4] = "caster_joint_fl";
        baseJointStateMessage.position[4] = 0.0;

        baseJointStateMessage.name[5] = "caster_joint_fr";
        baseJointStateMessage.position[5] = 0.0;

        baseJointStateMessage.name[6] = "caster_joint_bl";
        baseJointStateMessage.position[6] = 0.0;

        baseJointStateMessage.name[7] = "caster_joint_br";
        baseJointStateMessage.position[7] = 0.0;

        /*
         * Yet another hack to make the published values compatible with the URDF description.
         * We actually flipp the directions of the wheel on the right side such that the standard ROS controllers
         * (e.g. for PR2) can be used for the youBot
         */
        baseJointStateMessage.position[2] = -baseJointStateMessage.position[2];
        baseJointStateMessage.position[4] = -baseJointStateMessage.position[4];

    }

    if (youBotConfiguration.hasArms == true)
    {

        for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++)
        {
            ROS_ASSERT(youBotConfiguration.youBotArmConfigurations.size() == armJointStateMessages.size());

            /* fill joint state message */
            armJointStateMessages[armIndex].header.stamp = currentTime;
            armJointStateMessages[armIndex].name.resize(youBotArmDoF + youBotNumberOfFingers);
            armJointStateMessages[armIndex].position.resize(youBotArmDoF + youBotNumberOfFingers);
            armJointStateMessages[armIndex].velocity.resize(youBotArmDoF + youBotNumberOfFingers);

            if (youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm == 0)
            {
                ROS_ERROR("Arm%i is not correctly initialized! Cannot publish data.", armIndex + 1);
                continue;
            }

            ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex].jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
            for (int i = 0; i < youBotArmDoF; ++i)
            {
                youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getData(currentAngle); //youBot joints start with 1 not with 0 -> i + 1 //FIXME might segfault if only 1eout of 2 arms are initialized.
                youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).getData(currentVelocity);

                armJointStateMessages[armIndex].name[i] = youBotConfiguration.youBotArmConfigurations[armIndex].jointNames[i]; //TODO no unique names for URDF yet
                armJointStateMessages[armIndex].position[i] = currentAngle.angle.value();
                armJointStateMessages[armIndex].velocity[i] = currentVelocity.angularVelocity.value();
            }


            /*
             * NOTE: gripper slide rails are always symmetric, but the fingers can be screwed in different
             * positions! The published values account for the distance between the gripper slide rails, not the fingers
             * themselves. Of course if the finger are screwed to the most inner position (i.e. the can close completely),
             * than it is correct.
             */
            try 
            {
                youbot::YouBotGripperBar& gripperBar1 = youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar1();
                youbot::YouBotGripperBar& gripperBar2 = youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmGripper().getGripperBar2();

                if  (gripperCycleCounter == 0) { //workaround: avoid congestion of mailbox message by querying only every ith iteration
            	    gripperCycleCounter = youBotDriverCycleFrequencyInHz/5; //approx. 5Hz here
            	    gripperBar1.getData(gripperBar1Position);
            	    gripperBar2.getData(gripperBar2Position);
                }
                gripperCycleCounter--;

                armJointStateMessages[armIndex].name[youBotArmDoF + 0] = youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX];
                double leftGipperFingerPosition = gripperBar1Position.barPosition.value();
                armJointStateMessages[armIndex].position[youBotArmDoF + 0] = leftGipperFingerPosition;

                double rightGipperFingerPosition = gripperBar2Position.barPosition.value();
                armJointStateMessages[armIndex].name[youBotArmDoF + 1] = youBotConfiguration.youBotArmConfigurations[armIndex].gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX];
                armJointStateMessages[armIndex].position[youBotArmDoF + 1] = rightGipperFingerPosition;
            }
            catch (std::exception& e)
            {
                std::string errorMessage = e.what();
                ROS_WARN("Cannot read gripper values: \n %s", errorMessage.c_str());
            }

            if (trajectoryActionServerEnable)
            {
                // updating joint states in trajectory action 
                youBotConfiguration.youBotArmConfigurations[armIndex].jointTrajectoryAction->jointStateCallback(armJointStateMessages[armIndex]);
            }

        }
    }

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true); // ensure that all joint values will be send at the same time

}

void YouBotOODLWrapper::publishOODLSensorReadings()
{

    if (youBotConfiguration.hasBase)
    {
        youBotConfiguration.baseConfiguration.odometryBroadcaster.sendTransform(odometryTransform);
        youBotConfiguration.baseConfiguration.baseOdometryPublisher.publish(odometryMessage);
        youBotConfiguration.baseConfiguration.baseJointStatePublisher.publish(baseJointStateMessage);
    }

    if (youBotConfiguration.hasArms)
    {
        for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++)
        {
            youBotConfiguration.youBotArmConfigurations[armIndex].armJointStatePublisher.publish(armJointStateMessages[armIndex]);
        }
    }


}

bool YouBotOODLWrapper::switchOffBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	ROS_INFO("Switch off the base motors");
	if (youBotConfiguration.hasBase) { // in case stop has been invoked

		try {
			youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false); // ensure that all joint values will be send at the same time
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(1).noMoreAction();
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(2).noMoreAction();
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(3).noMoreAction();
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(4).noMoreAction();
			youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true); // ensure that all joint values will be send at the same time
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot switch off the base motors: \n %s", errorMessage.c_str());
			return false;
		}
	} else {
		ROS_ERROR("No base initialized!");
		return false;
	}
	return true;
}

bool YouBotOODLWrapper::switchOnBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Switch on the base motors");
    if (youBotConfiguration.hasBase)
    { // in case stop has been invoked
        quantity<si::velocity> longitudinalVelocity;
        quantity<si::velocity> transversalVelocity;
        quantity<si::angular_velocity> angularVelocity;

        longitudinalVelocity = 0.0 * meter_per_second;
        transversalVelocity = 0.0 * meter_per_second;
        angularVelocity = 0.0 * radian_per_second;

        try
        {
            youBotConfiguration.baseConfiguration.youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot set base velocities: \n %s", errorMessage.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("No base initialized!");
        return false;
    }
    return true;
}

bool YouBotOODLWrapper::switchOffArmMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex) {
	ROS_INFO("Switch off the arm%i motors", armIndex+1);
	ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));

	if (youBotConfiguration.hasArms && youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0) { // in case stop has been invoked

		try{
			youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false); // ensure that all joint values will be send at the same time
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(1).noMoreAction();
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(2).noMoreAction();
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(3).noMoreAction();
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(4).noMoreAction();
			youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(5).noMoreAction();
			youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true); // ensure that all joint values will be send at the same time
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot switch off the arm motors: \n %s", errorMessage.c_str());
			return false;
		}
	} else {
		ROS_ERROR("Arm%i not initialized!", armIndex+1);
		return false;
	}
	return true;
}

bool YouBotOODLWrapper::switchOnArmMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex)
{
    ROS_INFO("Switch on the arm%i motors", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.hasArms && youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0)
    {
        youbot::JointVelocitySetpoint desiredAngularVelocity;
        desiredAngularVelocity = 0.0 * radian_per_second;
        std::vector<youbot::JointVelocitySetpoint> desiredAngularVelocityVector;
        desiredAngularVelocityVector.assign(5, desiredAngularVelocity);
        try
        {
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->setJointData(desiredAngularVelocityVector);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot switch on the arm motors: \n %s", errorMessage.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("Arm%i not initialized!", armIndex + 1);
        return false;
    }
    return true;
}

bool YouBotOODLWrapper::calibrateArmCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex)
{
    ROS_INFO("Calibrate the arm%i", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    if (youBotConfiguration.hasArms && youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0)
    {

        try
        {
            youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->calibrateManipulator(true);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot calibrate the arm: \n %s", errorMessage.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("Arm%i not initialized!", armIndex + 1);
        return false;
    }
    return true;
}

bool YouBotOODLWrapper::reconnectCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

	this->stop();

	/* configuration */
	bool youBotHasBase;
	bool youBotHasArms;
	node.param("youBotHasBase", youBotHasBase, false);
	node.param("youBotHasArms", youBotHasArms, false);
	std::vector<std::string> armNames;
	std::string youBotBaseName;

	// Retrieve all defined arm names from the launch file params
	int i = 1;
	std::stringstream armNameParam;
	armNameParam << "youBotArmName" << i; // youBotArmName1 is first checked param... then youBotArmName2, etc.
	while (node.hasParam(armNameParam.str())) {
		std::string armName;
		node.getParam(armNameParam.str(), armName);
		armNames.push_back(armName);
		armNameParam.str("");
		armNameParam << "youBotArmName" <<  (++i);
	}

	ROS_ASSERT((youBotHasBase == true) || (youBotHasArms == true)); // At least one should be true, otherwise nothing to be started.
	if (youBotHasBase == true)
	{
		this->initializeBase(youBotBaseName);
	}

	if (youBotHasArms == true)
	{
		std::vector<std::string>::iterator armNameIter;
		for (armNameIter = armNames.begin(); armNameIter != armNames.end(); ++armNameIter) {
			this->initializeArm(*armNameIter);
		}
	}
	return true;
}

} // namespace youBot

/* EOF */

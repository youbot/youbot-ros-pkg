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
    areBaseMotorsSwitchedOn = false;	
    areArmMotorsSwitchedOn = false;

    youBotChildFrameID = "base_link"; //holds true for both: base and arm
    armJointStateMessages.clear();

    n.param("youBotDriverCycleFrequencyInHz", youBotDriverCycleFrequencyInHz, 50.0);
    //n.param("trajectoryActionServerEnable", trajectoryActionServerEnable, false);
    //n.param("trajectoryVelocityGain", trajectoryVelocityGain, 0.0);
    //n.param("trajectoryPositionGain", trajectoryPositionGain, 5.0);
    gripperCycleCounter = 0;
    diagnosticNameArms = "platform_Arms";
    diagnosticNameBase = "platform_Base";
    dashboardMessagePublisher = n.advertise<pr2_msgs::PowerBoardState>("/dashboard/platform_state", 1);
    diagnosticArrayPublisher = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
}

YouBotOODLWrapper::~YouBotOODLWrapper()
{
    this->stop();
    dashboardMessagePublisher.shutdown();
    diagnosticArrayPublisher.shutdown();
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
        ROS_FATAL("%s", errorMessage.c_str());
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
    areBaseMotorsSwitchedOn = true;
}

void YouBotOODLWrapper::initializeArm(std::string armName, std::string topic, bool enableStandardGripper)
{
    int armIndex;
    youbot::JointName jointNameParameter;
    std::string jointName;
    stringstream topicName;
    stringstream serviceName;

    YouBotArmConfiguration *armConfig = NULL;

    try
    {
        ROS_INFO("Configuration file path: %s", youBotConfiguration.configurationFilePath.c_str());   
        armConfig = new YouBotArmConfiguration();
        armIndex = static_cast<int> (youBotConfiguration.youBotArmConfigurations.size());
        armConfig->youBotArm = new youbot::YouBotManipulator(armName, youBotConfiguration.configurationFilePath);
        armConfig->armID = armName;
        armConfig->commandTopicName = topic + "/";
        armConfig->parentFrameIDName = "base_link";

        /* take joint names form configuration files */
        armConfig->jointNames.clear();
        for (int i = 0; i < youBotArmDoF; ++i)
        {
            armConfig->youBotArm->getArmJoint(i + 1).getConfigurationParameter(jointNameParameter);
            jointNameParameter.getParameter(jointName);
            armConfig->jointNames.push_back(jointName);
            ROS_INFO("Joint %i for arm %s has name: %s", i + 1, armConfig->armID.c_str(), armConfig->jointNames[i].c_str());
        }


        armConfig->youBotArm->doJointCommutation();
        armConfig->youBotArm->calibrateManipulator();
        if (enableStandardGripper)
        {
        	youbot::GripperBarName barName;
        	std::string gripperBarName;

            armConfig->youBotArm->getArmGripper().getGripperBar1().getConfigurationParameter(barName);
            barName.getParameter(gripperBarName);
            armConfig->gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX] = gripperBarName;
            ROS_INFO("Joint %i for gripper of arm %s has name: %s", 1, armConfig->armID.c_str(), gripperBarName.c_str());

            armConfig->youBotArm->getArmGripper().getGripperBar2().getConfigurationParameter(barName);
            barName.getParameter(gripperBarName);
            armConfig->gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX] = gripperBarName;
            ROS_INFO("Joint %i for gripper of arm %s has name: %s", 2, armConfig->armID.c_str(), gripperBarName.c_str());

            armConfig->youBotArm->calibrateGripper();

            // Add placeholder values to the gripperBarPosition buffers
            gripperBar1Position.push_back(youbot::GripperSensedBarPosition());
            gripperBar2Position.push_back(youbot::GripperSensedBarPosition());
        }

        // Get joint limits for each joint, and publish them in that arm's parameter namespace
        stringstream ss;
        for (int i = 0; i < youBotArmDoF; ++i) {
            bool inverseMovement;
            youbot::YouBotJoint &joint = armConfig->youBotArm->getArmJoint(i+1);
            youbot::JointLimitsRadian jointLimitsParam;
            youbot::InverseMovementDirection inverseDirectionParam;
              
            joint.getConfigurationParameter(jointLimitsParam);
            joint.getConfigurationParameter(inverseDirectionParam);

            inverseDirectionParam.getParameter(inverseMovement);

            quantity<plane_angle> lowerLimit, upperLimit;
            double lowerLimitVal, upperLimitVal;
            bool limitsActive;
            jointLimitsParam.getParameter(lowerLimit,upperLimit,limitsActive);

            lowerLimitVal = lowerLimit.value();
            upperLimitVal = upperLimit.value();

            if (inverseMovement) { 
                double temp = -upperLimitVal;
                upperLimitVal = -lowerLimitVal;
                lowerLimitVal = temp;
            }

            ss.str("");
            ss << armConfig->commandTopicName << "joint_limits/" << armConfig->jointNames[i].c_str() << "_lower";
            this->node.setParam(ss.str(), lowerLimitVal);

            ss.str("");
            ss << armConfig->commandTopicName << "joint_limits/" << armConfig->jointNames[i].c_str() << "_upper";
            this->node.setParam(ss.str(), upperLimitVal);
        }
        if (enableStandardGripper)
        {
            topicName.str("");
            topicName << armConfig->commandTopicName << "gripper_controller/position_command";
            armConfig->gripperPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions > (topicName.str(), 10, boost::bind(&YouBotOODLWrapper::gripperPositionsCommandCallback, this, _1, armIndex));
            armConfig->lastGripperCommand = 0.0; //This is true if the gripper is calibrated.

            double max_gripper_dist = 0.0115;
            const int GRIPPER_DOF = 2;
            for (int i=0; i<GRIPPER_DOF; ++i) {
                std::stringstream param_name;
                param_name << armConfig->commandTopicName << "joint_limits/" << armConfig->gripperFingerNames[i].c_str() << "_lower";
                this->node.setParam(param_name.str(), 0.);

                param_name.str("");      
                param_name << armConfig->commandTopicName << "joint_limits/" << armConfig->gripperFingerNames[i].c_str() << "_upper";
                this->node.setParam(param_name.str(), max_gripper_dist);
            }
        }
    }
    catch (std::exception& e)
    {
        if (armConfig != NULL) 
            delete armConfig;
        armConfig = NULL;
        std::string errorMessage = e.what();
        ROS_FATAL("%s", errorMessage.c_str());
        ROS_ERROR("Arm \"%s\" could not be initialized.", armName.c_str());
        ROS_INFO("System has %i initialized arm(s).", static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));
        return;
    }

    // Something went wrong in YouBotArmConfiguration initialization
    if (armConfig == NULL) {
        ROS_ERROR("Arm \"%s\" could not be initialized.", armName.c_str());
        return;
    }

    // Configuration initialized -- add to the list of arm configurations
    youBotConfiguration.youBotArmConfigurations.push_back(armConfig);

    /* setup input/output communication */
    topicName.str("");
    topicName << armConfig->commandTopicName << "arm_controller/position_command"; // e.g. arm_1/arm_controller/positionCommand
    armConfig->armPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::armPositionsCommandCallback, this, _1, armIndex));

    topicName.str("");
    topicName << armConfig->commandTopicName << "arm_controller/velocity_command";
    armConfig->armVelocityCommandSubscriber = node.subscribe<brics_actuator::JointVelocities > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::armVelocitiesCommandCallback, this, _1, armIndex));

	topicName.str("");
	topicName << armConfig->commandTopicName << "arm_controller/follow_joint_trajectory";
	// topicName.str("/arm_1/arm_controller/follow_joint_trajectory");
	armConfig->armJointTrajectoryAction = new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction > (
					node, topicName.str(),
					boost::bind(&YouBotOODLWrapper::armJointTrajectoryGoalCallback, this, _1, armIndex),
					boost::bind(&YouBotOODLWrapper::armJointTrajectoryCancelCallback, this, _1, armIndex), false);


    topicName.str("");
    topicName << armConfig->commandTopicName << "joint_states";
    armConfig->armJointStatePublisher = node.advertise<sensor_msgs::JointState > (topicName.str(), 1); //TODO different names or one topic?

    if (enableStandardGripper)
    {
        topicName.str("");
        topicName << armConfig->commandTopicName << "gripper_controller/position_command";
        armConfig->gripperPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::gripperPositionsCommandCallback, this, _1, armIndex));
        armConfig->lastGripperCommand = 0.0; //This is true if the gripper is calibrated.
    }



    /* setup services*/
    serviceName.str("");
    serviceName << armConfig->commandTopicName << "switchOffMotors"; // e.g. "arm_1/switchOffMotors"
    armConfig->switchOffMotorsService = node.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response > (serviceName.str(), boost::bind(&YouBotOODLWrapper::switchOffArmMotorsCallback, this, _1, _2, armIndex));

    serviceName.str("");
    serviceName << armConfig->commandTopicName << "switchOnMotors"; // e.g. "arm_1/switchOnMotors"
    armConfig->switchONMotorsService = node.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response > (serviceName.str(), boost::bind(&YouBotOODLWrapper::switchOnArmMotorsCallback, this, _1, _2, armIndex));

    serviceName.str("");
    serviceName << armConfig->commandTopicName << "calibrate"; // e.g. "arm_1/calibrate"
    armConfig->calibrateService = node.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response > (serviceName.str(), boost::bind(&YouBotOODLWrapper::calibrateArmCallback, this, _1, _2, armIndex));
/*
    if (trajectoryActionServerEnable)
    {
        JointStateObserver* jointStateObserver = new JointStateObserverOODL(this, armIndex);
        topicName.str("");
        topicName << armConfig->commandTopicName << "action";
        armConfig->jointTrajectoryAction = new JointTrajectoryAction(jointStateObserver,
                                                                                                                trajectoryPositionGain,
                                                                                                                trajectoryVelocityGain,
                                                                                                                youBotDriverCycleFrequencyInHz);

        serviceName.str("");
        serviceName << armConfig->commandTopicName << "arm_controller/joint_trajectory_action"; // e.g. "arm_1/switchOnMotors"
        
        armConfig->trajectoryActionServer = new Server(serviceName.str(),
                                                                                                  boost::bind(&YouBotOODLWrapper::executeActionServer, this, _1, armIndex),
                                                                                                  false);
        armConfig->trajectoryActionServer->start();
    }
*/
    /* initialize message vector for arm joint states */
    sensor_msgs::JointState dummyMessage;
    armJointStateMessages.push_back(dummyMessage);

    /* setup frame_ids */
    youBotArmFrameID = armName; // Set to something unique rather than something that may be re-used across multiple arms.
    ROS_INFO("Arm \"%s\" is initialized.", armName.c_str());
    ROS_INFO("System has %i initialized arm(s).", static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));
    youBotConfiguration.hasArms = true;
    areArmMotorsSwitchedOn = true;

    // currently no action is running
	armHasActiveJointTrajectoryGoal = false;

    //tracejoint = 4;
    //myTrace = new youbot::DataTrace(armConfig->youBotArm->getArmJoint(tracejoint), "Joint4TrajectoryTrace");

    // we can handle actionlib requests only after the complete initialization has been performed
	armConfig->armJointTrajectoryAction->start();
}

/*
void YouBotOODLWrapper::executeActionServer(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, int armIndex)
{

    JointTrajectoryAction* jointTrajectoryAction = armConfig->jointTrajectoryAction;
    if (jointTrajectoryAction != NULL)
    {

        jointTrajectoryAction->execute(goal, armConfig->trajectoryActionServer);
    }
}
*/
void YouBotOODLWrapper::stop()
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
    areBaseMotorsSwitchedOn = false;


    for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++) //delete each arm
    {
        YouBotArmConfiguration *armConfig = youBotConfiguration.youBotArmConfigurations[armIndex];
        ROS_ASSERT(armConfig != NULL);
        if (armConfig->youBotArm)
        {
            delete armConfig->youBotArm;
            armConfig->youBotArm = 0;
        }

        armConfig->armJointStatePublisher.shutdown();
        armConfig->armPositionCommandSubscriber.shutdown();
        armConfig->armVelocityCommandSubscriber.shutdown();
        armConfig->calibrateService.shutdown();
        armConfig->gripperPositionCommandSubscriber.shutdown();
        armConfig->switchONMotorsService.shutdown();
        armConfig->switchOffMotorsService.shutdown();
    }

    youBotConfiguration.hasArms = false;
    areArmMotorsSwitchedOn = false;
    youBotConfiguration.youBotArmConfigurations.clear();
    armJointStateMessages.clear();

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
            ROS_WARN("Cannot set base velocities: %s", errorMessage.c_str());
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

    YouBotArmConfiguration *armConfig = youBotConfiguration.youBotArmConfigurations[armIndex];
    ROS_ASSERT(armConfig != NULL);
    if (armConfig->youBotArm != 0) // in case stop has been invoked
    {

        ROS_DEBUG("Arm ID is: %s", armConfig->armID.c_str());
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
        ROS_ASSERT(armConfig->jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
        for (int i = 0; i < youBotArmDoF; ++i)
        {

            /* check what is in map */
            map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(armConfig->jointNames[i]);
            if (jointIterator != jointNameToValueMapping.end())
            {

                /* set the desired joint value */
                ROS_DEBUG("Trying to set joint %s to new position value %f", (armConfig->jointNames[i]).c_str(), jointIterator->second);
                desiredAngle.angle = jointIterator->second * radian;
                try
                {
                    armConfig->youBotArm->getArmJoint(i + 1).setData(desiredAngle); //youBot joints start with 1 not with 0 -> i + 1
                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: %s", i + 1, errorMessage.c_str());
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

    YouBotArmConfiguration *armConfig = youBotConfiguration.youBotArmConfigurations[armIndex];
    ROS_ASSERT(armConfig != NULL);
    if (armConfig->youBotArm != 0)
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
        ROS_ASSERT(armConfig->jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
        youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
        for (int i = 0; i < youBotArmDoF; ++i)
        {

            /* check what is in map */
            map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(armConfig->jointNames[i]);
            if (jointIterator != jointNameToValueMapping.end())
            {

                /* set the desired joint value */
                ROS_DEBUG("Trying to set joint %s to new velocity value %f", (armConfig->jointNames[i]).c_str(), jointIterator->second);
                desiredAngularVelocity.angularVelocity = jointIterator->second * radian_per_second;
                try
                {
                    armConfig->youBotArm->getArmJoint(i + 1).setData(desiredAngularVelocity); //youBot joints start with 1 not with 0 -> i + 1

                }
                catch (std::exception& e)
                {
                    std::string errorMessage = e.what();
                    ROS_WARN("Cannot set arm joint %i: %s", i + 1, errorMessage.c_str());
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


void YouBotOODLWrapper::armJointTrajectoryGoalCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal, unsigned int armIndex) {
	ROS_INFO("Goal for arm%i received", armIndex + 1);
	ROS_ASSERT(armIndex < youBotConfiguration.youBotArmConfigurations.size());

    YouBotArmConfiguration *armConfig = youBotConfiguration.youBotArmConfigurations[armIndex];
    ROS_ASSERT(armConfig != NULL);
	if (armConfig->youBotArm == 0) {
		ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
		youbotArmGoal.setRejected();
		return;
	}


	trajectory_msgs::JointTrajectory trajectory = youbotArmGoal.getGoal()->trajectory;

	// validate that the correct number of joints is provided in the goal
	if (trajectory.joint_names.size() != static_cast<unsigned int> (youBotArmDoF)) {
		ROS_ERROR("Trajectory is malformed! Goal has %i joint names, but only %i joints are supported", static_cast<int> (trajectory.joint_names.size()), youBotArmDoF);
		youbotArmGoal.setRejected();
		return;
	}

	// compare the joint names of the youBot configuration and joint names provided with the trajectory
	for (unsigned int i = 0; i < armConfig->jointNames.size(); i++) {
		bool jointNameFound = false;
		for (unsigned int j = 0; j < trajectory.joint_names.size(); j++) {
			if (armConfig->jointNames[i] == trajectory.joint_names[j]) {
				jointNameFound = true;
				break;
			}
		}

		if (!jointNameFound) {
			ROS_ERROR("Trajectory is malformed! Joint %s is missing in the goal", armConfig->jointNames[i].c_str());
			youbotArmGoal.setRejected();
			return;
		}
	}

  std::vector<youbot::JointTrajectory> jointTrajectories(youBotArmDoF);
  
	// convert from the ROS trajectory representation to the controller's representation
	std::vector<std::vector< quantity<plane_angle> > > positions(youBotArmDoF);
	std::vector<std::vector< quantity<angular_velocity> > > velocities(youBotArmDoF);
	std::vector<std::vector< quantity<angular_acceleration> > > accelerations(youBotArmDoF);
  youbot::TrajectorySegment segment;
	for (unsigned int i = 0; i < trajectory.points.size(); i++) {
		trajectory_msgs::JointTrajectoryPoint point = trajectory.points[i];
		// validate the trajectory point
		if ((point.positions.size() != static_cast<unsigned int> (youBotArmDoF)
						|| point.velocities.size() != static_cast<unsigned int> (youBotArmDoF)
						|| point.accelerations.size() != static_cast<unsigned int> (youBotArmDoF))) {
			ROS_ERROR("A trajectory point is malformed! %i positions, velocities and accelerations must be provided", youBotArmDoF);
			youbotArmGoal.setRejected();
			return;
		}
    
		for (int j = 0; j < youBotArmDoF; j++) {
      segment.positions = point.positions[j]*radian;
      segment.velocities = point.velocities[j]*radian_per_second;
      segment.accelerations = point.accelerations[j] * radian_per_second/second;
      segment.time_from_start = boost::posix_time::microsec(point.time_from_start.toNSec()/1000);
      jointTrajectories[j].segments.push_back(segment);
		}
	}
  for (int j = 0; j < youBotArmDoF; j++) {
      jointTrajectories[j].start_time = boost::posix_time::microsec_clock::local_time(); //TODO is this correct to set the trajectory start time to now
  }

  
  

	// cancel the old goal
  /*
	if (armHasActiveJointTrajectoryGoal) {
		armActiveJointTrajectoryGoal.setCanceled();
		armHasActiveJointTrajectoryGoal = false;
		for (int i = 0; i < youBotArmDoF; ++i) {
			armConfig->youBotArm->getArmJoint(i + 1).cancelTrajectory();
		}
	}
  */

	// replace the old goal with the new one
	youbotArmGoal.setAccepted();
	armActiveJointTrajectoryGoal = youbotArmGoal;
	armHasActiveJointTrajectoryGoal = true;

  
 // myTrace->startTrace();

	// send the trajectory to the controller
	for (int i = 0; i < youBotArmDoF; ++i) {
		try {
			// youBot joints start with 1 not with 0 -> i + 1
			armConfig->youBotArm->getArmJoint(i + 1).trajectoryController.setTrajectory(jointTrajectories[i]);
      ROS_INFO("set trajectories %d", i);
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot set trajectory for joint %i: %s", i + 1, errorMessage.c_str());
		}
	}
	ROS_INFO("set all trajectories");
}

void YouBotOODLWrapper::armJointTrajectoryCancelCallback(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle youbotArmGoal, unsigned int armIndex) {
	ROS_DEBUG("Goal for arm%i received", armIndex + 1);
	ROS_ASSERT(armIndex < youBotConfiguration.youBotArmConfigurations.size());

    YouBotArmConfiguration *armConfig = youBotConfiguration.youBotArmConfigurations[armIndex];
    ROS_ASSERT(armConfig != NULL);

	// stop the controller
	for (int i = 0; i < youBotArmDoF; ++i) {
		try {
			// youBot joints start with 1 not with 0 -> i + 1
      //TODO cancel trajectory
			armConfig->youBotArm->getArmJoint(i + 1).trajectoryController.cancelCurrentTrajectory();
			armConfig->youBotArm->getArmJoint(i + 1).stopJoint();
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot stop joint %i: %s", i + 1, errorMessage.c_str());
		}
	}

	if (armActiveJointTrajectoryGoal == youbotArmGoal) {
		// Marks the current goal as canceled.
		youbotArmGoal.setCanceled();
		armHasActiveJointTrajectoryGoal = false;
	}
}

void YouBotOODLWrapper::gripperPositionsCommandCallback(const brics_actuator::JointPositionsConstPtr& youbotGripperCommand, int armIndex)
{
	ROS_DEBUG("Command for gripper%i received", armIndex + 1);
	ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));

    YouBotArmConfiguration *armConfig = youBotConfiguration.youBotArmConfigurations[armIndex];
    ROS_ASSERT(armConfig != NULL);
	if (armConfig->youBotArm != 0) { // in case stop has been invoked

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
		gripperIterator = jointNameToValueMapping.find(armConfig->gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX]);
		if (gripperIterator != jointNameToValueMapping.end()) {
			ROS_DEBUG("Trying to set the left gripper finger to new value %f", gripperIterator->second);

			leftGripperFingerPosition.barPosition = gripperIterator->second * meter;
			try {
				armConfig->youBotArm->getArmGripper().getGripperBar1().setData(leftGripperFingerPosition);
			} catch (std::exception& e) {
				std::string errorMessage = e.what();
				ROS_WARN("Cannot set the left gripper finger: %s", errorMessage.c_str());
			}
		}

		/* check if something is in the received message that requires action for the right finger gripper */
		gripperIterator = jointNameToValueMapping.find(armConfig->gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX]);
		if (gripperIterator != jointNameToValueMapping.end()) {
			ROS_DEBUG("Trying to set the right gripper to new value %f", gripperIterator->second);

			rightGripperFingerPosition.barPosition = gripperIterator->second * meter;
			try {
				armConfig->youBotArm->getArmGripper().getGripperBar2().setData(rightGripperFingerPosition);
			} catch (std::exception& e) {
				std::string errorMessage = e.what();
				ROS_WARN("Cannot set the right gripper finger: %s", errorMessage.c_str());
			}
		}

		youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
	} else {
		ROS_ERROR("Arm%i is not correctly initialized!", armIndex + 1);
	}
}

void YouBotOODLWrapper::computeOODLSensorReadings()
{

	try{
    currentTime = ros::Time::now();
    youbot::JointSensedAngle currentAngle;
    youbot::JointSensedVelocity currentVelocity;

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false); // ensure that all joint values will be received at the same time

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
        baseJointStateMessage.position[0] = -baseJointStateMessage.position[0];
        baseJointStateMessage.position[2] = -baseJointStateMessage.position[2];

    }

    if (youBotConfiguration.hasArms == true)
    {

        bool queryGripperPositions = false;

        --gripperCycleCounter;
        if (gripperCycleCounter <= 0) {
            queryGripperPositions = true;
            gripperCycleCounter = youBotDriverCycleFrequencyInHz/5;
        }

        for (int armIndex = 0; armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()); armIndex++)
        {
            ROS_ASSERT(youBotConfiguration.youBotArmConfigurations.size() == armJointStateMessages.size());

            YouBotArmConfiguration *armConfig = youBotConfiguration.youBotArmConfigurations[armIndex];
            ROS_ASSERT(armConfig != NULL);

            /* fill joint state message */
            armJointStateMessages[armIndex].header.stamp = currentTime;
            armJointStateMessages[armIndex].name.resize(youBotArmDoF + youBotNumberOfFingers);
            armJointStateMessages[armIndex].position.resize(youBotArmDoF + youBotNumberOfFingers);
            armJointStateMessages[armIndex].velocity.resize(youBotArmDoF + youBotNumberOfFingers);

            if (armConfig->youBotArm == 0)
            {
                ROS_ERROR("Arm%i is not correctly initialized! Cannot publish data.", armIndex + 1);
                continue;
            }

            ROS_ASSERT(armConfig->jointNames.size() == static_cast<unsigned int> (youBotArmDoF));
            for (int i = 0; i < youBotArmDoF; ++i)
            {
                armConfig->youBotArm->getArmJoint(i + 1).getData(currentAngle); //youBot joints start with 1 not with 0 -> i + 1 //FIXME might segfault if only 1eout of 2 arms are initialized.
                armConfig->youBotArm->getArmJoint(i + 1).getData(currentVelocity);

                armJointStateMessages[armIndex].name[i] = armConfig->jointNames[i]; //TODO no unique names for URDF yet
                armJointStateMessages[armIndex].position[i] = currentAngle.angle.value();
                armJointStateMessages[armIndex].velocity[i] = currentVelocity.angularVelocity.value();
            }

            // check if trajectory controller is finished
			bool areTrajectoryControllersDone = true;
			for (int i = 0; i < youBotArmDoF; ++i) {
				if (armConfig->youBotArm->getArmJoint(i + 1).trajectoryController.isTrajectoryControllerActive()) {
					areTrajectoryControllersDone = false;
					break;
				}
			}
			if (areTrajectoryControllersDone && armHasActiveJointTrajectoryGoal) {
                armHasActiveJointTrajectoryGoal = false;
				control_msgs::FollowJointTrajectoryResult trajectoryResult;
				trajectoryResult.error_code = trajectoryResult.SUCCESSFUL;
				armActiveJointTrajectoryGoal.setSucceeded(trajectoryResult, "trajectory successful");
                // ROS_INFO("trajectory successful");
                // myTrace->stopTrace();
                // myTrace->plotTrace();
			}

            /*
             * NOTE: gripper slide rails are always symmetric, but the fingers can be screwed in different
             * positions! The published values account for the distance between the gripper slide rails, not the fingers
             * themselves. Of course if the finger are screwed to the most inner position (i.e. the can close completely),
             * than it is correct.
             */
            try 
            {
                youbot::YouBotGripperBar& gripperBar1 = armConfig->youBotArm->getArmGripper().getGripperBar1();
                youbot::YouBotGripperBar& gripperBar2 = armConfig->youBotArm->getArmGripper().getGripperBar2();

                if  (queryGripperPositions == true) { //workaround: avoid congestion of mailbox message by querying only every ith iteration
            	    gripperCycleCounter = youBotDriverCycleFrequencyInHz/5; //approx. 5Hz here
            	    gripperBar1.getData(gripperBar1Position[armIndex]);
            	    gripperBar2.getData(gripperBar2Position[armIndex]);
                }

                double leftGripperFingerPosition = gripperBar1Position[armIndex].barPosition.value();
                armJointStateMessages[armIndex].name[youBotArmDoF + 0] = armConfig->gripperFingerNames[YouBotArmConfiguration::LEFT_FINGER_INDEX];
                armJointStateMessages[armIndex].position[youBotArmDoF + 0] = leftGripperFingerPosition;

                double rightGripperFingerPosition = gripperBar2Position[armIndex].barPosition.value();
                armJointStateMessages[armIndex].name[youBotArmDoF + 1] = armConfig->gripperFingerNames[YouBotArmConfiguration::RIGHT_FINGER_INDEX];
                armJointStateMessages[armIndex].position[youBotArmDoF + 1] = rightGripperFingerPosition;
            }
            catch (std::exception& e)
            {
                std::string errorMessage = e.what();
                ROS_WARN("Cannot read gripper values: %s", errorMessage.c_str());
            }
/*
            if (trajectoryActionServerEnable)
            {
                // updating joint states in trajectory action 
                armConfig->jointTrajectoryAction->jointStateCallback(armJointStateMessages[armIndex]);
            }
*/
        }
    }

    youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true); // ensure that all joint values will be received at the same time
  }catch (youbot::EtherCATConnectionException& e)
  {
      ROS_WARN("%s", e.what());
      youBotConfiguration.hasBase = false;
      youBotConfiguration.hasArms = false;
  }
	catch (std::exception& e)
	{
		ROS_WARN_ONCE("%s", e.what());
	}

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
            ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex] != NULL);
            youBotConfiguration.youBotArmConfigurations[armIndex]->armJointStatePublisher.publish(armJointStateMessages[armIndex]);
        }
    }


}

bool YouBotOODLWrapper::switchOffBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
	ROS_INFO("Switch off the base motors");
	if (youBotConfiguration.hasBase) { // in case stop has been invoked

        youbot::JointCurrentSetpoint currentStopMovement;
        currentStopMovement.current = 0.0 * ampere;
		try {
      youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(1).setData(currentStopMovement);
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(2).setData(currentStopMovement);
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(3).setData(currentStopMovement);
			youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(4).setData(currentStopMovement);
      youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot switch off the base motors: %s", errorMessage.c_str());
			return false;
		}
	} else {
		ROS_ERROR("No base initialized!");
		return false;
	}
  	areBaseMotorsSwitchedOn = false;
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
            ROS_WARN("Cannot set base velocities: %s", errorMessage.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("No base initialized!");
        return false;
    }
    areBaseMotorsSwitchedOn = true;
    return true;
}

bool YouBotOODLWrapper::switchOffArmMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex) {
	ROS_INFO("Switch off the arm%i motors", armIndex+1);
	ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));

    YouBotArmConfiguration *armConfig = youBotConfiguration.youBotArmConfigurations[armIndex];
    ROS_ASSERT(armConfig != NULL);
	if (youBotConfiguration.hasArms && armConfig->youBotArm != 0) { // in case stop has been invoked

        youbot::JointCurrentSetpoint currentStopMovement;
        currentStopMovement.current = 0.0 * ampere;
		try{
      youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
			armConfig->youBotArm->getArmJoint(1).setData(currentStopMovement);
			armConfig->youBotArm->getArmJoint(2).setData(currentStopMovement);
			armConfig->youBotArm->getArmJoint(3).setData(currentStopMovement);
			armConfig->youBotArm->getArmJoint(4).setData(currentStopMovement);
			armConfig->youBotArm->getArmJoint(5).setData(currentStopMovement);
      youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot switch off the arm motors: %s", errorMessage.c_str());
			return false;
		}
	} else {
		ROS_ERROR("Arm%i not initialized!", armIndex+1);
		return false;
	}
  	areArmMotorsSwitchedOn = false;
	return true;
}

bool YouBotOODLWrapper::switchOnArmMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex)
{
    ROS_INFO("Switch on the arm%i motors", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    YouBotArmConfiguration *armConfig = youBotConfiguration.youBotArmConfigurations[armIndex];
    ROS_ASSERT(armConfig != NULL);
    if (youBotConfiguration.hasArms && armConfig->youBotArm != 0)
    {   
      try
        {
            std::vector<youbot::JointSensedAngle> sensedJointAngleVector;
            std::vector<youbot::JointAngleSetpoint> desiredJointAngleVector;
            armConfig->youBotArm->getJointData(sensedJointAngleVector);
            youbot::JointAngleSetpoint desiredJointAngle;
            for(unsigned int i = 0; i < sensedJointAngleVector.size(); i++){
              desiredJointAngle = sensedJointAngleVector[i].angle;
              desiredJointAngleVector.push_back(desiredJointAngle);
            }        
            armConfig->youBotArm->setJointData(desiredJointAngleVector);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot switch on the arm motors: %s", errorMessage.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("Arm%i not initialized!", armIndex + 1);
        return false;
    }
    areArmMotorsSwitchedOn = true;
    return true;
}

bool YouBotOODLWrapper::calibrateArmCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response, int armIndex)
{
    ROS_INFO("Calibrate the arm%i", armIndex + 1);
    ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int> (youBotConfiguration.youBotArmConfigurations.size()));

    YouBotArmConfiguration *armConfig = youBotConfiguration.youBotArmConfigurations[armIndex];
    ROS_ASSERT(armConfig != NULL);
    if (youBotConfiguration.hasArms && armConfig->youBotArm != 0)
    {

        try
        {
            armConfig->youBotArm->calibrateManipulator(true);
        }
        catch (std::exception& e)
        {
            std::string errorMessage = e.what();
            ROS_WARN("Cannot calibrate the arm: %s", errorMessage.c_str());
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
        this->initializeBase(this->youBotConfiguration.baseConfiguration.baseID);
    }

    if (youBotHasArms == true) {
        for (unsigned int i=0; i<armNames.size(); i++) {
            // determine topic name for arm #i
            std::string topic;
            std::stringstream armTopicParam;
            armTopicParam << "youBotArmTopic" << i+1; // youBotArmTopic1, youBotArmName2, etc.
            if (node.hasParam(armTopicParam.str())) {
                node.getParam(armTopicParam.str(), topic);
            } else {
                // if no parameter is provided, use "arm_1", "arm_2" etc. as default topic names
                std::stringstream armTopicDefault;
                armTopicDefault << "arm_" << i+1;
                topic = armTopicDefault.str();
            }
            initializeArm(armNames[i], topic);
        }
    }

    return true;
}

void YouBotOODLWrapper::publishArmAndBaseDiagnostics(double publish_rate_in_secs) {
    // only publish every X seconds
    if ((ros::Time::now() - lastDiagnosticPublishTime).toSec() < publish_rate_in_secs)
      return;

    lastDiagnosticPublishTime = ros::Time::now();

    diagnosticArrayMessage.header.stamp = ros::Time::now();
    diagnosticArrayMessage.status.clear();

    // diagnostics message
    diagnosticStatusMessage.name = "platform_Base";
    if (youBotConfiguration.hasBase) {
      diagnosticStatusMessage.message = "base is present";
      diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::OK;
    } else {
      diagnosticStatusMessage.message = "base is not connected or switched off";
      diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }

    diagnosticArrayMessage.status.push_back(diagnosticStatusMessage);

    diagnosticStatusMessage.name = "platform_Arm";
    if (youBotConfiguration.hasArms) {
      diagnosticStatusMessage.message = "arm is present";
      diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::OK;
    } else {
      diagnosticStatusMessage.message = "arm is not connected or switched off";
      diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    }

    diagnosticArrayMessage.status.push_back(diagnosticStatusMessage);


    // dashboard message
    platformStateMessage.header.stamp = ros::Time::now();

    if (youBotConfiguration.hasBase && areBaseMotorsSwitchedOn)
      platformStateMessage.circuit_state[0] = pr2_msgs::PowerBoardState::STATE_ENABLED;
    else if (youBotConfiguration.hasBase && !areBaseMotorsSwitchedOn)
      platformStateMessage.circuit_state[0] = pr2_msgs::PowerBoardState::STATE_STANDBY;
    else
      platformStateMessage.circuit_state[0] = pr2_msgs::PowerBoardState::STATE_DISABLED;

    if (youBotConfiguration.hasArms && areArmMotorsSwitchedOn)
      platformStateMessage.circuit_state[1] = pr2_msgs::PowerBoardState::STATE_ENABLED;
    else if (youBotConfiguration.hasArms && !areArmMotorsSwitchedOn)
      platformStateMessage.circuit_state[1] = pr2_msgs::PowerBoardState::STATE_STANDBY;
    else
      platformStateMessage.circuit_state[1] = pr2_msgs::PowerBoardState::STATE_DISABLED;


    // publish established messages
    dashboardMessagePublisher.publish(platformStateMessage);
    diagnosticArrayPublisher.publish(diagnosticArrayMessage);
  }

} // namespace youBot

/* EOF */

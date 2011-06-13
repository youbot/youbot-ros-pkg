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
#include <sstream>

namespace youBot {


YouBotOODLWrapper::YouBotOODLWrapper() {

}

YouBotOODLWrapper::YouBotOODLWrapper(ros::NodeHandle n) :
	node(n)
{
	youBotBase = 0;
	youBotArm = 0;
	hasBase = false;
	hasArm = false;

	youBotChildFrameID = "base_link"; //holds true for both: base and arm

	/* setup the names */

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

	jointNames.clear();
	jointNames.push_back("arm_joint_1");
	jointNames.push_back("arm_joint_2");
	jointNames.push_back("arm_joint_3");
	jointNames.push_back("arm_joint_4");
	jointNames.push_back("arm_joint_5");

	gripperJointName = "gripper_joint";

	gripperFingerNames.clear();
	gripperFingerNames.push_back("gripper_finger_joint_l");
	gripperFingerNames.push_back("gripper_finger_joint_r");

}

YouBotOODLWrapper::~YouBotOODLWrapper() {
	this->stop();
}

void YouBotOODLWrapper::initializeBase() {

	try {
		youBotBase = new youbot::YouBotBase("youbot-base", mkstr(YOUBOT_CONFIGURATIONS_DIR));
		youBotBase->doJointCommutation();
//	} catch (youbot::FileNotFoundException& e) {
	} catch (std::exception& e) {
		std::string errorMessage = e.what();
		ROS_FATAL("Cannot open youBot driver: \n %s ", errorMessage.c_str());
		ROS_ERROR("Base could not be initialized!");
		hasBase = false;
		return;
	}

	/* setup input/output communication */
	baseCommandSubscriber = node.subscribe("cmd_vel", 1000, &YouBotOODLWrapper::baseCommandCallback, this);
	baseOdometryPublisher = node.advertise<nav_msgs::Odometry>("odom", 1);
	baseJointStatePublisher = node.advertise<sensor_msgs::JointState>("base_joint_states", 1);

	/* setup frame_ids */
	youBotOdometryFrameID = "odom";
	youBotOdometryChildFrameID = "base_footprint";

	ROS_INFO("Base is initialized.");
	hasBase = true;
}

void YouBotOODLWrapper::initializeArm() {

	try {
		youBotArm = new youbot::YouBotManipulator("youbot-manipulator", mkstr(YOUBOT_CONFIGURATIONS_DIR));
		youBotArm->doJointCommutation();
		youBotArm->calibrateManipulator();
//	} catch (youbot::FileNotFoundException& e) {
	} catch (std::exception& e) {
		std::string errorMessage = e.what();
		ROS_FATAL("Cannot open youBot driver: \n %s ", errorMessage.c_str());
		ROS_ERROR("Arm could not be initialized!");
		hasArm = false;
		return;
//		exit(-1);
	}

	/* (optional) set all joints into velocity mode -> so the arm can be manually moved */
	youbot::JointVelocitySetpoint jointVelocity;
	for (int i = 0; i < youBotArmDoF; ++i) {
		jointVelocity.angularVelocity = 0.0 * radian_per_second;
		try {
			youBotArm->getArmJoint(i+1).setData(jointVelocity);
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot set arm velocity %i: \n %s", i+1, errorMessage.c_str());
		}
	}

	/* setup input/output communication */
	armCommandSubscriber = node.subscribe("arm_controller/command", 1000, &YouBotOODLWrapper::armCommandCallback, this);
	armJointStatePublisher = node.advertise<sensor_msgs::JointState>("joint_states", 1);

	/* setup frame_ids */
	youBotArmFrameID = "arm";  //TODO find default topic name

	lastGripperCommand = 0.0; //This is true if the gripper is calibrated.

	ROS_INFO("Arm is initialized.");
	hasArm = true;
}

void YouBotOODLWrapper::initializeArm(std::string armName, bool enableStandardGripper) {
	youbot::JointName jointNameParameter;
	std::string jointName;

	try {
		ROS_INFO("Configuration file path: %s", youBotConfiguration.configurationFilePath.c_str());
		YouBotArmConfiguration tmpArmConfig;
		youBotConfiguration.youBotArmConfigurations.push_back(tmpArmConfig);
		youBotConfiguration.youBotArmConfigurations.back().youBotArm = new youbot::YouBotManipulator(armName, youBotConfiguration.configurationFilePath);
		youBotConfiguration.youBotArmConfigurations.back().armID = armName;
		youBotConfiguration.youBotArmConfigurations.back().commandTopicName = armName; // arm_1/arm_controller/
		youBotConfiguration.youBotArmConfigurations.back().parentFrameIDName = armName; //?
		youBotConfiguration.armNameToArmIndexMapping.insert(make_pair(armName, static_cast<int>(youBotConfiguration.youBotArmConfigurations.size())));

		for (int i = 0; i < youBotArmDoF; ++i) {
			youBotConfiguration.youBotArmConfigurations.back().youBotArm->getArmJoint(i+1).getConfigurationParameter(jointNameParameter);
			jointNameParameter.getParameter(jointName);
			youBotConfiguration.youBotArmConfigurations.back().jointNameToJointIndexMapping.insert(make_pair(jointName, i+1));
			ROS_INFO("Joint %i for arm %s has name: %s", i+1, armName.c_str(), jointName.c_str()); //TODO DEBUG
		}


		youBotConfiguration.youBotArmConfigurations.back().youBotArm->doJointCommutation();
		youBotConfiguration.youBotArmConfigurations.back().youBotArm->calibrateManipulator();
		if(enableStandardGripper) {
			youBotConfiguration.youBotArmConfigurations.back().youBotArm->calibrateGripper();
		}
	} catch (std::exception& e) {
		youBotConfiguration.youBotArmConfigurations.pop_back();
		std::string errorMessage = e.what();
		ROS_FATAL("Cannot open youBot driver: \n %s ", errorMessage.c_str());
		ROS_ERROR("Arm could not be initialized!");
		ROS_INFO("System has %i initialized arms.", static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));
		hasArm = false;
		return;
	}

	/* (optional) set all joints into velocity mode -> so the arm can be manually moved */
	youbot::JointVelocitySetpoint jointVelocity;
	for (int i = 0; i < youBotArmDoF; ++i) {
		jointVelocity.angularVelocity = 0.0 * radian_per_second;
		try {
			youBotConfiguration.youBotArmConfigurations.back().youBotArm->getArmJoint(i+1).setData(jointVelocity);
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot set arm velocity %i: \n %s", i+1, errorMessage.c_str());
		}
	}

	/* setup input/output communication */
	std::string topicName = youBotConfiguration.youBotArmConfigurations.back().commandTopicName; //TODO
	topicName += "/arm_controller/positionCommand";
	int armIndex = static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()) - 1;

//	youBotConfiguration.youBotArmConfigurations.back().armPositionCommandSubscriber = node.subscribe("arm_controller/positionCommand", 1000, &YouBotOODLWrapper::armPositionsCommandCallback, this);
	youBotConfiguration.youBotArmConfigurations.back().armPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions>("arm_controller/positionCommand", 1000, boost::bind(&YouBotOODLWrapper::armPositionsCommandCallback, this, _1, armIndex));


//	youBotConfiguration.youBotArmConfigurations.back().armPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions>("arm1/arm_controller/positionCommand", 1000, boost::bind(&YouBotOODLWrapper::armPositionsCommandCallback2, this, _1, armIndex));
//	n.subscribe<std_msgs::String>(topics[i], 1000,
//	boost::bind(&MyClass::callback, this, _1, topics[i]))

	youBotConfiguration.youBotArmConfigurations.back().armVelocityCommandSubscriber = node.subscribe("arm_controller/velocityCommand", 1000, &YouBotOODLWrapper::armVelocitiesCommandCallback, this);
//	youBotConfiguration.youBotArmConfigurations.back().armVelocityCommandSubscriber = node.subscribe<brics_actuator::JointVelocities>("arm_controller/velocityCommand", 1000, boost::bind(&YouBotOODLWrapper::armVelocitiesCommandCallback, this, _1, armIndex));
	armJointStatePublisher = node.advertise<sensor_msgs::JointState>("joint_states", 1);

	if(enableStandardGripper) {
		youBotConfiguration.youBotArmConfigurations.back().gripperPositionCommandSubscriber = node.subscribe("gripper_controller/positionCommand", 1000, &YouBotOODLWrapper::gripperPositionsCommandCallback, this);
//		youBotConfiguration.youBotArmConfigurations.back().gripperPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions>("gripper_controller/positionCommand", 1000, boost::bind(&YouBotOODLWrapper::gripperPositionsCommandCallback, this, _1, armIndex));
		lastGripperCommand = 0.0; //This is true if the gripper is calibrated.
	}

	/* setup frame_ids */
	youBotArmFrameID = "arm";  //TODO find default topic name



	ROS_INFO("Arm \"%s\" is initialized.", armName.c_str());
	ROS_INFO("System has %i initialized arms.", static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));
	youBotConfiguration.hasArms = true;
	hasArm = true;
}

void YouBotOODLWrapper::stop() {

	if (hasBase) {
		if (youBotBase) {
			delete youBotBase;
			youBotBase = 0;
		}
		hasBase = false;
	}

	if (hasArm) {
		if (youBotArm) {
			delete youBotArm;
			youBotArm = 0;
		}
		hasArm = false;
	}

}

void YouBotOODLWrapper::baseCommandCallback(const geometry_msgs::Twist& youbotBaseCommand){

	if (hasBase) { // in case stop has been invoked
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

		longitudinalVelocity =  youbotBaseCommand.linear.x * meter_per_second;
		transversalVelocity = youbotBaseCommand.linear.y * meter_per_second;
		angularVelocity = youbotBaseCommand.angular.z * radian_per_second;

		try {
			youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
		} catch (std::exception& e) {
			std::string errorMessage = e.what();
			ROS_WARN("Cannot set base velocities: \n %s", errorMessage.c_str());
		}

	} else {
		ROS_ERROR("No base initialized!");
	}
}

void YouBotOODLWrapper::armCommandCallback(const trajectory_msgs::JointTrajectory& youbotArmCommand) {
//
//	if (hasArm) { // in case stop has been invoked
//
//		trajectory_msgs::JointTrajectoryPoint nextDesiredConfiguration;
//		if (youbotArmCommand.points.size() < 1){
//			ROS_ERROR("youBot driver received an invalid joint trajectory command.");
//			return;
//		}
//
//		nextDesiredConfiguration = youbotArmCommand.points[0];
//		youbot::JointAngleSetpoint desiredAngle;
//
//		if (nextDesiredConfiguration.positions.size() != youbotArmCommand.joint_names.size()){
//			ROS_ERROR("youBot driver received an invalid joint trajectory command.");
//			return;
//		}
//		if (nextDesiredConfiguration.positions.size() < static_cast<unsigned int>(youBotArmDoF)){
//			ROS_ERROR("youBot driver received an invalid joint trajectory command.");
//			return;
//		}
//
//		/* populate mapping between joint names and values  */
//		std::map<string, double> jointNameToValueMapping;
//		for (int i = 0; i < static_cast<int>(youbotArmCommand.joint_names.size()); ++i) {
//			jointNameToValueMapping.insert(make_pair(youbotArmCommand.joint_names[i], nextDesiredConfiguration.positions[i]));
//		}
//
//		/* loop over all youBot arm joints and check if something is in the received message that requires action */
//		ROS_ASSERT(jointNames.size() == static_cast<unsigned int>(youBotArmDoF));
//		for (int i = 0; i < youBotArmDoF; ++i) {
//
//			/* check what is in map */
//			map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(jointNames[i]);
//			if (jointIterator != jointNameToValueMapping.end()) {
//
//				/* set the desired joint value */
//				ROS_DEBUG("Trying to set joint %s to new value %f", (jointNames[i]).c_str(), jointIterator->second);
//				desiredAngle.angle = jointIterator->second * radian;
//				try {
//					youBotConfiguration.youBotArmConfigurations[0].youBotArm->getArmJoint(i + 1).setData(desiredAngle); //youBot joints start with 1 not with 0 -> i + 1
//				} catch (std::exception& e) {
//					std::string errorMessage = e.what();
//					ROS_WARN("Cannot set arm joint %i: \n %s", i+1, errorMessage.c_str());
//				}
//			}
//		}
//
//		/* check if there is something in in the message for the gripper */
//		map<string, double>::const_iterator gripperIterator = jointNameToValueMapping.find(gripperJointName);
//		if (gripperIterator != jointNameToValueMapping.end()) {
//			ROS_DEBUG("Trying to set the gripper to new value %f", gripperIterator->second);
//			youbot::GripperBarSpacingSetPoint gripperSlideRailDistance;
//
//			gripperSlideRailDistance.barSpacing = gripperIterator->second * meter;
//
//			try {
//				youBotConfiguration.youBotArmConfigurations[0].youBotArm->getArmGripper().setData(gripperSlideRailDistance);
//				lastGripperCommand = gripperSlideRailDistance.barSpacing.value();
//			} catch (std::exception& e) {
//				std::string errorMessage = e.what();
//				ROS_WARN("Cannot set the gripper: \n %s", errorMessage.c_str());
//			}
//		}
//
//	} else {
//		ROS_ERROR("No arm initialized!");
//	}
}

//void YouBotOODLWrapper::armPositionsCommandCallback2(const ros::MessageEvent<brics_actuator::JointPositions const>& youbotArmCommand, const int& armIndex) {
//	ROS_WARN("Not yet implemented for armIndex %i!", armIndex);
//}

void YouBotOODLWrapper::armPositionsCommandCallback(const brics_actuator::JointPositionsConstPtr& youbotArmCommand, int armIndex) {
	ROS_WARN("Command for arm%i received", armIndex+1);
	ROS_ASSERT(0 <= armIndex && armIndex < static_cast<int>(youBotConfiguration.youBotArmConfigurations.size()));
	ROS_ASSERT(youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm != 0);


	if (youbotArmCommand->positions.size() < 1){
		ROS_WARN("youBot driver received an invalid joint positions command.");
		return;
	}

	youbot::JointAngleSetpoint desiredAngle;

	/* populate mapping between joint names and values  */
	std::map<string, double> jointNameToValueMapping;
	for (int i = 0; i < static_cast<int>(youbotArmCommand->positions.size()); ++i) {
		jointNameToValueMapping.insert(make_pair(youbotArmCommand->positions[i].joint_uri, youbotArmCommand->positions[i].value));
	}

	/* loop over all youBot arm joints and check if something is in the received message that requires action */
	ROS_ASSERT(jointNames.size() == static_cast<unsigned int>(youBotArmDoF));
	for (int i = 0; i < youBotArmDoF; ++i) {

		/* check what is in map */
		map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(jointNames[i]);
		if (jointIterator != jointNameToValueMapping.end()) {

			/* set the desired joint value */
			ROS_DEBUG("Trying to set joint %s to new position value %f", (jointNames[i]).c_str(), jointIterator->second);
			desiredAngle.angle = jointIterator->second * radian;
			try {
				youBotConfiguration.youBotArmConfigurations[armIndex].youBotArm->getArmJoint(i + 1).setData(desiredAngle); //youBot joints start with 1 not with 0 -> i + 1
			} catch (std::exception& e) {
				std::string errorMessage = e.what();
				ROS_WARN("Cannot set arm joint %i: \n %s", i+1, errorMessage.c_str());
			}
		}
	}

}

void YouBotOODLWrapper::armVelocitiesCommandCallback(const brics_actuator::JointVelocities& youbotArmCommand/*, const int& armIndex*/) {
	if (hasArm) { // in case stop has been invoked

		if (youbotArmCommand.velocities.size() < 1){
			ROS_WARN("youBot driver received an invalid joint velocities command.");
			return;
		}

		youbot::JointVelocitySetpoint desiredAngularVelocity;

		/* populate mapping between joint names and values  */
		std::map<string, double> jointNameToValueMapping;
		for (int i = 0; i < static_cast<int>(youbotArmCommand.velocities.size()); ++i) {
			jointNameToValueMapping.insert(make_pair(youbotArmCommand.velocities[i].joint_uri, youbotArmCommand.velocities[i].value));
		}

		/* loop over all youBot arm joints and check if something is in the received message that requires action */
		ROS_ASSERT(jointNames.size() == static_cast<unsigned int>(youBotArmDoF));
		for (int i = 0; i < youBotArmDoF; ++i) {

			/* check what is in map */
			map<string, double>::const_iterator jointIterator = jointNameToValueMapping.find(jointNames[i]);
			if (jointIterator != jointNameToValueMapping.end()) {

				/* set the desired joint value */
				ROS_DEBUG("Trying to set joint %s to new velocity value %f", (jointNames[i]).c_str(), jointIterator->second);
				desiredAngularVelocity.angularVelocity = jointIterator->second * radian_per_second;
				try {
					youBotConfiguration.youBotArmConfigurations[0].youBotArm->getArmJoint(i + 1).setData(desiredAngularVelocity); //youBot joints start with 1 not with 0 -> i + 1
				} catch (std::exception& e) {
					std::string errorMessage = e.what();
					ROS_WARN("Cannot set arm joint %i: \n %s", i+1, errorMessage.c_str());
				}
			}
		}
	} else {
		ROS_ERROR("No arm initialized!");
	}
}

void YouBotOODLWrapper::gripperPositionsCommandCallback(const brics_actuator::JointPositions& youbotArmCommand/*, const int& armIndex*/){ //TODO rename
	if (hasArm) { // in case stop has been invoked

		if (youbotArmCommand.positions.size() < 1){
			ROS_WARN("youBot driver received an invalid gripper positions command.");
			return;
		}
		youbot::GripperBarSpacingSetPoint gripperSlideRailDistance;
		gripperSlideRailDistance.barSpacing = 0 * meter;
		bool validGripperCommandReceived = false;

		/* populate mapping between joint names and values  */
		std::map<string, double> jointNameToValueMapping;
		for (int i = 0; i < static_cast<int>(youbotArmCommand.positions.size()); ++i) {
			jointNameToValueMapping.insert(make_pair(youbotArmCommand.positions[i].joint_uri, youbotArmCommand.positions[i].value));
		}

		/* loop over all youBot gripper joints and check if something is in the received message that requires action */
		ROS_ASSERT(gripperFingerNames.size() == static_cast<unsigned int>(youBotNumberOfFingers));
		for (int i = 0; i < youBotNumberOfFingers; ++i) {

			/* check if there is something in in the message for the gripper */
			map<string, double>::const_iterator gripperIterator = jointNameToValueMapping.find(gripperFingerNames[i]);
			if (gripperIterator != jointNameToValueMapping.end()) {
				ROS_DEBUG("Trying to set the gripper to new value %f", gripperIterator->second);

				gripperSlideRailDistance.barSpacing += gripperIterator->second * meter; //for now just stack all values, as long youBot oodl API does not suport two fingers
				validGripperCommandReceived = true;
			}
		}
		if (validGripperCommandReceived) { // at least one valid command received that requires action (set accumulated gripper value)
			try {
				youBotConfiguration.youBotArmConfigurations[0].youBotArm->getArmGripper().setData(gripperSlideRailDistance);
				lastGripperCommand = gripperSlideRailDistance.barSpacing.value();
			} catch (std::exception& e) {
				std::string errorMessage = e.what();
				ROS_WARN("Cannot set the gripper: \n %s", errorMessage.c_str());
			}
		}
	} else {
		ROS_ERROR("No arm initialized!");
	}
}

void YouBotOODLWrapper::computeOODLSensorReadings() {

	currentTime = ros::Time::now();
    youbot::JointSensedAngle currentAngle;
    youbot::JointSensedVelocity currentVelocity;

	if (hasBase == true) {
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

		youBotBase->getBasePosition(longitudinalPosition, transversalPosition, orientation);
		x = longitudinalPosition.value();
		y = transversalPosition.value();
		theta = orientation.value();

		youBotBase->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
		vx = longitudinalVelocity.value();
		vy = transversalVelocity.value();
		vtheta = angularVelocity.value();
		ROS_DEBUG("Perceived odometric values (x,y,tetha, vx,vy,vtetha): %f, %f, %f \t %f, %f, %f", x, y, theta, vx, vy, vtheta);


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
		odometryMessage.twist.twist.linear.x =  vx;
		odometryMessage.twist.twist.linear.y = vy;
		odometryMessage.twist.twist.angular.z = vtheta;

		/* Set up joint state message for the wheels */
		baseJointStateMessage.header.stamp = currentTime;
		baseJointStateMessage.name.resize(youBotNumberOfWheels * 2); // *2 because of virtual wheel joints in the URDF description
		baseJointStateMessage.position.resize(youBotNumberOfWheels * 2);
		baseJointStateMessage.velocity.resize(youBotNumberOfWheels * 2);

		ROS_ASSERT((wheelNames.size() == static_cast<unsigned int>(youBotNumberOfWheels)));
		for (int i = 0; i < youBotNumberOfWheels; ++i) {
			youBotBase->getBaseJoint(i + 1).getData(currentAngle); //youBot joints start with 1 not with 0 -> i + 1
			youBotBase->getBaseJoint(i + 1).getData(currentVelocity);

			baseJointStateMessage.name[i] = wheelNames[i];
			baseJointStateMessage.position[i] = currentAngle.angle.value();
			baseJointStateMessage.velocity[i] = currentVelocity.angularVelocity.value();
		}

		/*
		 * TODO Here we add values for "virtual" rotation joints in URDF - robot_state_publisher can't
		 * handle non-aggregated jointState messages well ...
		 */
		baseJointStateMessage.name[4] = "rotation_joint_fl";
		baseJointStateMessage.position[4] = 0.0;

		baseJointStateMessage.name[5] = "rotation_joint_fr";
		baseJointStateMessage.position[5] = 0.0;

		baseJointStateMessage.name[6] = "rotation_joint_bl";
		baseJointStateMessage.position[6] = 0.0;

		baseJointStateMessage.name[7] = "rotation_joint_br";
		baseJointStateMessage.position[7] = 0.0;

	}

	if (hasArm == true) {

		/* fill joint state message */
		jointStateMessage.header.stamp = currentTime;
		jointStateMessage.name.resize(youBotArmDoF + youBotNumberOfFingers);
		jointStateMessage.position.resize(youBotArmDoF + youBotNumberOfFingers);
		jointStateMessage.velocity.resize(youBotArmDoF + youBotNumberOfFingers);

		ROS_ASSERT(jointNames.size() == static_cast<unsigned int>(youBotArmDoF));
		for (int i = 0; i < youBotArmDoF; ++i) {
			youBotConfiguration.youBotArmConfigurations[0].youBotArm->getArmJoint(i + 1).getData(currentAngle); //youBot joints start with 1 not with 0 -> i + 1
			youBotConfiguration.youBotArmConfigurations[0].youBotArm->getArmJoint(i + 1).getData(currentVelocity);

			jointStateMessage.name[i] = jointNames[i];
			jointStateMessage.position[i] = currentAngle.angle.value();
			jointStateMessage.velocity[i] = currentVelocity.angularVelocity.value();
		}

		/*
		 * NOTE: gripper slide rails are always symmetric, but the fingers can be screwed in different
		 * positions! The published values account for the distance between the gripper slide rails, not the fingers
		 * themselves. Of course if the finger are screwed to the most inner position (i.e. the can close completely),
		 * than it is correct.
		 */
		youbot::GripperBarSpacingSetPoint gripperSlideRailDistance;
//		youBotArm->getArmGripper().getData(gripperSlideRailDistance); // this is not yet implemented in OODL
//		double distance = gripperSlideRailDistance.barSpacing.value();

		ROS_ASSERT(gripperFingerNames.size() == static_cast<unsigned int>(youBotNumberOfFingers));
		for (int i = 0; i < youBotNumberOfFingers; ++i) {

			jointStateMessage.name[youBotArmDoF + i] = gripperFingerNames[i];
			jointStateMessage.position[youBotArmDoF + i] = lastGripperCommand / 2; //as the distance is symmetric, each finger travels half of the distance
		}



	}


}

void YouBotOODLWrapper::publishOODLSensorReadings() {

	if (hasBase) {
			odometryBroadcaster.sendTransform(odometryTransform);
			baseOdometryPublisher.publish(odometryMessage);
			baseJointStatePublisher.publish(baseJointStateMessage);
	}

	if (hasArm) {
		armJointStatePublisher.publish(jointStateMessage); //depends on order... if baseJointStatePublisher and armJointStatePublisher publish to same topic
	}


}

}  // namespace youBot

/* EOF */

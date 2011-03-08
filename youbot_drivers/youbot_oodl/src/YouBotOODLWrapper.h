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

#ifndef YOUBOTOODLWRAPPER_H_
#define YOUBOTOODLWRAPPER_H_

/* ROS includes */
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"

/* OODL includes */
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

namespace youBot {

/**
 * @brief Wrapper class to map ROS messages to OODL method calls for the youBot platform.
 */
class YouBotOODLWrapper {
public:

	/**
	 * @brief Constructor with a ROS handle.
	 * @param n ROS handle
	 */
	YouBotOODLWrapper(ros::NodeHandle n);

	/**
	 * @brief DEfault constructor.
	 */
	virtual ~YouBotOODLWrapper();


	/* Coordination: */

	/**
	 * @brief Initializes a youBot base.
	 */
	void initializeBase();

	/**
	 * @brief Initializes a youBot arm.
	 */
	void initializeArm();

	/**
	 * @brief Stops all initialized elements.
	 * Stops arm and/or base (if initialized).
	 */
	void stop();

	/* Configuration: */


	/* Communication: */

	/**
	 * @brief Callback that is executed when a commend for the base comes in.
	 * @param youbotBaseCommand Message that contains the desired translational and rotational velocity for the base.
	 */
	void baseCommandCallback(const geometry_msgs::Twist& youbotBaseCommand);

	/**
	 * @brief Callback that is executed when a commend for the arm comes in.
	 * @param youbotArmCommand Message that contains the desired joint configuration.
	 *
	 * Currently only the first configuration (JointTrajectoryPoint) per message is processed.
	 * Velocity and acceleration values are ignored.
	 */
	void armCommandCallback(const trajectory_msgs::JointTrajectory& youbotArmCommand);

	/**
	 * @brief Publishes all sensor measurements. Both for base and arm.
	 *
	 * Depending on what has been initialized before, either odometry and/or joint state valiues are published.
	 * computeOODLSensorReadings needs to be executed before.
	 */
	void publishOODLSensorReadings();

	/* Computation: */

	/**
	 * @brief Mapps OODL values to ROS messages
	 */
	void computeOODLSensorReadings();

private:

	YouBotOODLWrapper(); //forbid default constructor

	///Flag to indicate if youBot has a base (set after successful initialization)
	bool hasBase;

	///Flag to indicate if youBot has an arm (set after successful initialization)
	bool hasArm;


	/// Degrees of freedom for the youBot manipulator
	static const int youBotArmDoF = 5;

	/// Number of finger mounted on the gripper.
	static const int youBotNumberOfFingers = 2;

	/// Number of wheels attached to the base.
	static const int youBotNumberOfWheels = 4;

	/**
	 * This variable memorizes the last successfully set value for the gripper,
	 * so it can be published in the joint state message. This is necessary at the moment, as
	 * it is not yet possible to measure the actual distance. Consider the gripper joint state
	 * as an open loop value.
	 */
	double lastGripperCommand;


    /// Handle to the base
	youbot::YouBotBase* youBotBase;

	/// Handle to the arm
	youbot::YouBotManipulator* youBotArm;

	/// Path to the configuration files, required by OODL (e.g. youbot-base.cfg)
	std::string configurationFilePath;


	std::string youBotChildFrameID;
	std::string youBotOdometryFrameID;
	std::string youBotOdometryChildFrameID;
	std::string youBotArmFrameID;

	std::vector<std::string> wheelNames;
	std::vector<std::string> jointNames;
	std::string gripperJointName;
	std::vector<std::string> gripperFingerNames;


	/// The ROS node handle
	ros::NodeHandle node;

	/// ROS timestamp
	ros::Time currentTime;


	/**
	 * Receives JointTrajectory messages for the arm.
	 * Currently only the first configuration (JointTrajectoryPoint) per message is processed.
	 */
	ros::Subscriber baseCommandSubscriber;

	/// Receives Twist messages for the base.
	ros::Subscriber armCommandSubscriber;

	/// Publishes Odometry messages
	ros::Publisher baseOdometryPublisher;

	/// Publishes JointState messages with angles/velocities for the wheels.
	ros::Publisher baseJointStatePublisher;

	/// Publishes JointState messages with angles for the arm.
	ros::Publisher armJointStatePublisher;

	/// Puglishes tf frames as odomery
	tf::TransformBroadcaster odometryBroadcaster;



	/// The published odometry message with distances in [m], angles in [RAD] and velocities in [m/s] and [RAD/s]
	nav_msgs::Odometry odometryMessage;

	/// The published odometry tf frame with distances in [m]
	geometry_msgs::TransformStamped odometryTransform;

	/// The quaternion inside the tf odometry frame with distances in [m]
	geometry_msgs::Quaternion odometryQuaternion;

	/// The published joint state of the base (wheels) with angles in [RAD] and velocities in [RAD/s]
	sensor_msgs::JointState baseJointStateMessage;

	/// The published joint state of the arm with angles in [RAD]
	sensor_msgs::JointState jointStateMessage; //TODO rename

};

}  // namespace youBot

#endif /* YOUBOTOODLWRAPPER_H_ */

/* EOF */

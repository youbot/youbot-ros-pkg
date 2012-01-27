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

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

/* ROS includes */
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "sensor_msgs/JointState.h"

#include "brics_actuator/JointPositions.h"
#include "brics_actuator/JointVelocities.h"

/* OODL includes */
#include "YouBotConfiguration.h"

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
	 * @param baseName Name of the base. Used to open the configuration file e.g. youbot-base.cfg
	 */
	void initializeBase(std::string baseName);

	/**
	 * @brief Initializes a youBot base.
	 * @param armName Name of the base. Used to open the configuration file e.g. youbot-manipulator.cfg
	 * @param enableStandardGripper If set to true, then the default gripper of the youBot will be initialized.
	 */
	void initializeArm(std::string armName, bool enableStandardGripper = true);

	/**
	 * @brief Stops all initialized elements.
	 * Stops arm and/or base (if initialized).
	 */
	void stop();


	/* Communication: */

	/**
	 * @brief Callback that is executed when a commend for the base comes in.
	 * @param youbotBaseCommand Message that contains the desired translational and rotational velocity for the base.
	 */
	void baseCommandCallback(const geometry_msgs::Twist& youbotBaseCommand);

	/**
	 * @deprecated
	 * @brief Callback that is executed when a commend for the arm comes in.
	 * @param youbotArmCommand Message that contains the desired joint configuration.
	 *
	 * Currently only the first configuration (JointTrajectoryPoint) per message is processed.
	 * Velocity and acceleration values are ignored.
	 */
	void armCommandCallback(const trajectory_msgs::JointTrajectory& youbotArmCommand);

	/**
	 * @brief Callback that is executed when a position command for the arm comes in.
	 * @param youbotArmCommand Message that contains the desired joint configuration.
	 * @param armIndex Index that identifies the arm
	 */
	void armPositionsCommandCallback(const brics_actuator::JointPositionsConstPtr& youbotArmCommand, int armIndex);

	/**
	 * @brief Callback that is executed when a velocity command for the arm comes in.
	 * @param youbotArmCommand Message that contains the desired joint configuration.
	 * @param armIndex Index that identifies the arm
	 */
	void armVelocitiesCommandCallback(const brics_actuator::JointVelocitiesConstPtr& youbotArmCommand, int armIndex);

	/**
	 * @brief Callback that is executed when a position command for the gripper comes in.
	 * @param youbotGripperCommand Message that contains the desired joint configuration.
	 * @param armIndex Index that identifies the arm
	 */
	void gripperPositionsCommandCallback(const brics_actuator::JointPositionsConstPtr& youbotGripperCommand, int armIndex);

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
  
  bool switchOffBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool switchOnBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  
  bool switchOffArm1MotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool switchOnArm1MotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  
  bool calibrateArm1Callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  

	/* Configuration: */

	/// Handle the aggregates all parts of a youBot system
	YouBotConfiguration youBotConfiguration;

private:

	YouBotOODLWrapper(); //forbid default constructor


	/// Degrees of freedom for the youBot manipulator
	static const int youBotArmDoF = 5;

	/// Number of finger mounted on the gripper.
	static const int youBotNumberOfFingers = 2;

	/// Number of wheels attached to the base.
	static const int youBotNumberOfWheels = 4;


	std::string youBotChildFrameID;
	std::string youBotOdometryFrameID;
	std::string youBotOdometryChildFrameID;
	std::string youBotArmFrameID;


	/// The ROS node handle
	ros::NodeHandle node;

	/// ROS timestamp
	ros::Time currentTime;


	/// The published odometry message with distances in [m], angles in [RAD] and velocities in [m/s] and [RAD/s]
	nav_msgs::Odometry odometryMessage;

	/// The published odometry tf frame with distances in [m]
	geometry_msgs::TransformStamped odometryTransform;

	/// The quaternion inside the tf odometry frame with distances in [m]
	geometry_msgs::Quaternion odometryQuaternion;

	/// The published joint state of the base (wheels) with angles in [RAD] and velocities in [RAD/s]
	sensor_msgs::JointState baseJointStateMessage;

	/// Vector of the published joint states of per arm with angles in [RAD]
	vector<sensor_msgs::JointState> armJointStateMessages;

};

}  // namespace youBot

#endif /* YOUBOTOODLWRAPPER_H_ */

/* EOF */

/******************************************************************************
 * Copyright (c) 2011
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
 * * Neither the name of company nor the names of its
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
#include <boost/units/physical_dimensions.hpp>
#include <boost/units/io.hpp>
#include <geometry_msgs/Quaternion.h>

#include "cartesian_compliance_control.h"
#include "20_sim_cartesian_compliance_control/common/xxmatrix.h"
#include "pluginlib/class_list_macros.h"
#include <tf/transform_datatypes.h>

PLUGINLIB_DECLARE_CLASS(youbot_description, CartesianComplianceController, controller::CartesianComplianceController, pr2_controller_interface::Controller)


using namespace std;

namespace controller {

CartesianComplianceController* CartesianComplianceControllerDebug::getControllerPtr() {
	return dynamic_cast<CartesianComplianceController*> (this->controllerPtr);
}

void CartesianComplianceControllerDebug::toTransformMatrix(double* tf, tf::Transform& trans) {
	trans.setOrigin(tf::Vector3(tf[3], tf[7], tf[11]));
	tf::Matrix3x3 rotMatrix(tf[0], tf[1], tf[2],
			tf[4], tf[5], tf[6],
			tf[8], tf[9], tf[10]);
	tf::Quaternion quat;
	rotMatrix.getRotation(quat);
	trans.setRotation(quat);
}

void CartesianComplianceControllerDebug::publishTf(double* tf, string parent, string child) {
	/*tf::Transform trans;
	trans.setOrigin(tf::Vector3(tf[3], tf[7], tf[11]));
	tf::Matrix3x3 rotMatrix(tf[0], tf[1], tf[2],
			tf[4], tf[5], tf[6],
			tf[8], tf[9], tf[10]);
	tf::Quaternion quat;
	rotMatrix.getRotation(quat);
	trans.setRotation(quat);*/
	tf::Transform trans;
	toTransformMatrix(tf, trans);
	br.sendTransform(tf::StampedTransform(trans, ros::Time::now(), parent, child));
}

void CartesianComplianceControllerDebug::init() {

	ros::NodeHandle &nodeHandle = getControllerPtr()->nodeHandle;
/*	gazeboJointPositions.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointPositions > (nodeHandle, "gazebo_joint_positions", 1));
	gazeboJointPositions->lock();
	controllerJointPositions.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointPositions > (nodeHandle, "controller_joint_positions", 1));
	controllerJointPositions->lock();
	gazeboJointTorques.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointTorques > (nodeHandle, "gazebo_joint_torques", 1));
	gazeboJointTorques->lock();
	controllerJointTorques.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointTorques > (nodeHandle, "controller_joint_torques", 1));
	controllerJointTorques->lock();
	gazeboJointPose.reset(new realtime_tools::RealtimePublisher<brics_actuator::CartesianPose > (nodeHandle, "gazebo_joint_pose", 1));
	gazeboJointPose->lock();
	controllerJointPose.reset(new realtime_tools::RealtimePublisher<brics_actuator::CartesianPose > (nodeHandle, "controller_joint_pose", 1));
	controllerJointPose->lock();
*/
    currentTipPose.reset(new realtime_tools::RealtimePublisher<brics_actuator::CartesianPose> (nodeHandle, "current_tip_pose", 1));
    targetTipPose.reset(new realtime_tools::RealtimePublisher<brics_actuator::CartesianPose> (nodeHandle, "target_tip_pose", 1));
}

void CartesianComplianceControllerDebug::publish() {

	/*vector <brics_actuator::JointValue> positionsGazebo;
	vector <brics_actuator::JointValue> positionsController;
	vector <brics_actuator::JointValue> torquesGazebo;*/

	CartesianComplianceController* ctrlPtr = getControllerPtr();

	/*unsigned int size = ctrlPtr->joints.size();
	positionsGazebo.resize(size);
	positionsController.resize(size);
	torquesGazebo.resize(size);*/


	XXMatrix* matrix = ctrlPtr->autoGenerated20simController.m_M;
	double* joint1 = matrix[42].mat;
	publishTf(joint1, "/odom", "/joint1");
	double* joint2 = matrix[43].mat;
	publishTf(joint2, "/odom", "/joint2");
	double* joint3 = matrix[44].mat;
	publishTf(joint3, "/odom", "/joint3");
	double* joint4 = matrix[45].mat;
	publishTf(joint4, "/odom", "/joint4");
	double* joint5 = matrix[46].mat;
	publishTf(joint5, "/odom", "/joint5");
	double* tip = matrix[2].mat;
	publishTf(tip, "/odom", "/tip");

//	std::vector<pr2_mechanism_model::JointState*>::iterator joinsIterator;

	using namespace boost::units;


//	unsigned int j = 0;

/*	for (joinsIterator = ctrlPtr->joints.begin(); joinsIterator != ctrlPtr->joints.end(); ++joinsIterator) {
		positionsGazebo[j].joint_uri = (*joinsIterator)->joint_->name;
		positionsGazebo[j].value = (*joinsIterator)->position_;
		positionsGazebo[j].unit = to_string(si::radians);
		positionsGazebo[j].timeStamp = ctrlPtr->currentTime;

		positionsController[j].joint_uri = (*joinsIterator)->joint_->name;
		//positionsController[j].value = u[28+j]; //CHANGE THAT!
		positionsController[j].unit = to_string(si::radians);
		positionsController[j].timeStamp = ctrlPtr->currentTime;

		torquesGazebo[j].joint_uri = (*joinsIterator)->joint_->name;
		torquesGazebo[j].value = (*joinsIterator)->commanded_effort_;
		torquesGazebo[j].unit = to_string(si::newton_meter);
		torquesGazebo[j].timeStamp = ctrlPtr->currentTime;
		++j;
	}

	gazeboJointPositions->msg_.positions = positionsGazebo;
	controllerJointPositions->msg_.positions = positionsController;
	gazeboJointTorques->msg_.torques = torquesGazebo;

	gazeboJointPositions->unlockAndPublish();
	controllerJointPositions->unlockAndPublish();
	gazeboJointTorques->unlockAndPublish();
*/
//    tf::Transform trans;
//	toTransformMatrix(tf, trans);



    currentTipPose->msg_ = ctrlPtr->currentTipPose;
    targetTipPose->msg_ = ctrlPtr->targetTipPose;
    currentTipPose->unlockAndPublish();
    targetTipPose->unlockAndPublish();
}

CartesianComplianceController::CartesianComplianceController()
: robotPtr(NULL) {
    locked = true;
	loopCount = 0;
	this->position[0] = 0;
	this->position[1] = 0;
	this->position[2] = 0;

	this->orientationYPR[0] = 0;
	this->orientationYPR[1] = 0;
	this->orientationYPR[2] = 0;
#ifdef DEBUG_INFO
	debugInfo = new CartesianComplianceControllerDebug(this);

#else
	debugInfo = new Debug(this);
#endif

}

CartesianComplianceController::~CartesianComplianceController() {
	subscriber.shutdown();
	autoGenerated20simController.Terminate(u, y);
	delete debugInfo;
}

/*  auto-generated by 20sim;
	initialize the inputs and outputs with correct initial values */
void CartesianComplianceController::init20SimController() {

	/* initialize the inputs and outputs with correct initial values */
	//angles of the joionts
	u[0] = 0.0; //is not active
	u[1] = 0.0; //is not active
	u[2] = 0.0; //is not active
	u[3] = -170 * M_PI / 180; //J1
	u[4] = -65 * M_PI / 180; //J2
	u[5] = -146 * M_PI / 180; //J3
	u[6] = -102.5 * M_PI / 180; //J4
	u[7] = -167.5 * M_PI / 180; //J5
	/* velocities of the joints used for active dumping*/
	u[8] = 0.0; //is not active
	u[9] = 0.0; //is not active
	u[10] = 0.0; //is not active
	u[11] = 0.0; //J1
	u[12] = 0.0; //J2
	u[13] = 0.0; //J3
	u[14] = 0.0; //J4
	u[15] = 0.0; //J5
	u[16] = 0.0; /* xyzrpy *///not tested
	u[17] = 0.0;
	u[18] = 0.53;
	u[19] = 0.0;
	u[20] = 0.0;
	u[21] = 0.0;

	y[0] = 0.0; /* output */
	y[1] = 0.0;
	y[2] = 0.0;
	y[3] = 0.0;
	y[4] = 0.0;
	y[5] = 0.0;
	y[6] = 0.0;
	y[7] = 0.0;
	y[8] = 0.0;
	y[9] = 0.0;
	y[10] = 0.0;
	y[11] = 0.0;
	y[12] = 0.0;
	y[13] = 0.0;
	y[14] = 0.0;
	y[15] = 0.0;
	y[16] = 0.0; /* p.e */
	y[17] = 0.0;
	y[18] = 0.0;
	y[19] = 0.0;
	y[20] = 0.0;
	y[21] = 0.0;
	y[22] = 0.0;
	y[23] = 0.0;

	ROS_INFO("20sim init.\n");
	autoGenerated20simController.Initialize(u, y, 0.0);

}

bool CartesianComplianceController::init(pr2_mechanism_model::RobotState *robotPtr, ros::NodeHandle &nodeHandle) {
	using namespace XmlRpc;
	this->nodeHandle = nodeHandle;
	this->robotPtr = robotPtr;

	ROS_INFO("Initializing interaction control for the youbot arm...\n");

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

	for (unsigned int i = 0; i < static_cast<unsigned int> (jointNames.size()); ++i) {
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

	// Initializing target efforts vector
	targetEfforts.resize(joints.size());

	// Subscribing for an input pose command
	subscriber = nodeHandle.subscribe("command", 1, &CartesianComplianceController::positionCommand, this);

	// Initializing 20Sim controller
	init20SimController();

	// Initializing twist publisher for the base
	ROS_INFO("base_confghfhgtroller/command\n");
	baseTwist.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist > (nodeHandle, "base_controller/command", 1));
	baseTwist->lock();

	subscriberOdometry = nodeHandle.subscribe("base_odometry/odometry", 1, &CartesianComplianceController::odometryCommand, this);


	// Initializing debug output
	debugInfo->init();

	return true;
}

void CartesianComplianceController::starting() {
	// Initializing timer
	currentTime = robotPtr->getTime();
	lastTime = currentTime;
}

void CartesianComplianceController::update20SimControl() {

    if (!locked) {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(currentBasePose.orientation, quat);

        double ypr[3];
        tf::Matrix3x3(quat).getEulerYPR(ypr[0], ypr[1], ypr[2]);

        u[0] = ypr[0]; //is not active
     //  ROS_INFO("X: %f\n", u[0]);
    }

	u[1] = currentBasePose.position.x; //is not active
	u[2] = currentBasePose.position.y; //is not active
	u[3] = joints[0]->position_ - 170 * M_PI / 180; // -170 +170, clockwise
	u[4] = joints[1]->position_ - 65 * M_PI / 180; // -65  +90,  counterclockwis
	u[5] = -joints[2]->position_ - 146 * M_PI / 180; // +146 -151, clockwise
	//ROS_INFO("joint 3: %f", u[5]);
	u[6] = joints[3]->position_ - 102.5 * M_PI / 180; // -102 +102, counterclockwise
	u[7] = joints[4]->position_ - 167.5 * M_PI / 180; // -167 +167, clockwise
	/* velocities of the joints used for active dumping*/

	u[8] = 0.0;//currentBaseTwist.angular.z; //is not active
	u[9] = 0.0;//currentBaseTwist.linear.y;//0.0; //is not active
	u[10] = 0.0;//currentBaseTwist.linear.x;//0.0; //is not active
	//ROS_INFO("currentBaseTwist: x=%f, y=%f, z=%f\n", u[10], u[9], u[8]);

	u[11] = 0;//joints[0]->velocity_; //J1
	u[12] = 0;//joints[1]->velocity_; //J2
	u[13] = 0;//joints[2]->velocity_; //J3
	u[14] = 0;//joints[3]->velocity_; //J4
	u[15] = 0;//joints[4]->velocity_; //J5

	u[16] = this->position[0]; //0;0.0;            /* xyzYawPitchRoll *///not tested
	u[17] = this->position[1]; //0;0.2;
	u[18] = this->position[2]; //0,53;0.4;
	u[19] = this->orientationYPR[0];
	u[20] = this->orientationYPR[1]; //1.57;1.57;
	u[21] = this->orientationYPR[2]; //0.7;-0.7;-0.7

	autoGenerated20simController.Calculate(u, y);

	targetSpeed[0] = y[16]; // rot
	targetSpeed[1] = y[17]; // y
	targetSpeed[2] = y[18]; // x
    ROS_INFO("joint 3: %f", y[21]);
	targetEfforts[0] = y[19]; // -170 +170, clockwise
	targetEfforts[1] = y[20]; // -65  +90,  counterclockwise
	targetEfforts[2] = y[21]; // +146 -151, clockwise
	targetEfforts[3] = y[22]; // -102 +102, counterclockwise
	targetEfforts[4] = y[23]; // -167 +167, clockwise

}

void CartesianComplianceController::update() {



	currentTime = robotPtr->getTime();
	ros::Duration dt = currentTime - lastTime;
	lastTime = currentTime;

	// Initializing error vector
	if (!locked) //{
		update20SimControl();
	//ROS_INFO ("Current speed is: %f, %f\n", currentBaseTwist.angular.z, currentBaseTwist.linear.y);

	// } else
	//   ROS_INFO ("locked\n");

  //  ROS_INFO("vel: %f, %f, %f, %f, %f\n",joints[0]->velocity_,joints[1]->velocity_,joints[2]->velocity_,joints[3]->velocity_,joints[4]->velocity_);
	// Doing control here, calculating and applying the efforts
	for (unsigned int i = 0; i < joints.size(); ++i) {
		joints[i]->commanded_effort_ += targetEfforts[i] * 10.0;
	}

	// Sending control comands for the base

	//gazeboJointPositions->msg_.positions = positionsGazebo;
	//baseTwist->msg_ = ;
	geometry_msgs::Twist twist;
	baseTwist->trylock();


    twist.linear.x = targetSpeed[2] / 15.0;
    twist.linear.y = targetSpeed[1] / 15.0;
	//twist.angular.x = 0.0;//targetSpeed[2];
    //twist.angular.y = 0.0;//targetSpeed[1];
    twist.angular.z = targetSpeed[0] / 15.0 ;

	//ROS_INFO("Speed: %f, %f, %f\n", twist.angular.x, twist.angular.y, twist.angular.z);

	baseTwist->msg_ = twist;
	baseTwist->unlockAndPublish();

	// if debug mode is set, publishing debug TFs and controller information
	if (loopCount % 10 == 0) {
		debugInfo->publish();
	}
	++loopCount;
}

void CartesianComplianceController::positionCommand(const brics_actuator::CartesianPose &pose) {

	using namespace boost::units;

    targetTipPose = pose;

	brics_actuator::CartesianVector tipPosition;
	geometry_msgs::Quaternion tipOrientation;
	tipPosition = pose.position;
	tipOrientation = pose.orientation;

	if (tipPosition.unit != to_string(si::meter))
		ROS_ERROR("Position value is set in the inpcompatible units %s, expecting meters", tipPosition.unit.c_str());

	this->position[0] = tipPosition.x;
	this->position[1] = tipPosition.y;
	this->position[2] = tipPosition.z;

	tf::Quaternion quaternion;
	tf::quaternionMsgToTF(tipOrientation, quaternion);
	tf::Matrix3x3(quaternion).getEulerYPR(orientationYPR[0], orientationYPR[1], orientationYPR[2]);

}

void CartesianComplianceController::odometryCommand(const nav_msgs::Odometry &odometry) {
  //  ROS_INFO ("Odom: %f, %f, %f\n", currentBasePose.position.x, currentBasePose.position.y, currentBasePose.position.z);
  //  ROS_INFO ("Odom: %f, %f, %f, %f\n", currentBasePose.orientation.x, currentBasePose.orientation.y, currentBasePose.orientation.z,  currentBasePose.orientation.w);
	locked = true;
	currentBaseTwist = odometry.twist.twist;
	currentBasePose = odometry.pose.pose;
	locked = false;

}

}

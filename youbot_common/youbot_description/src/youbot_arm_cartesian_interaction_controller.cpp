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


#include <boost/units/systems/si.hpp>
#include <boost/units/physical_dimensions.hpp>
#include <boost/units/io.hpp>
//#include <tf/transform_broadcaster.h>

#include "youbot_arm_cartesian_interaction_controller.h"

#include <tf/transform_datatypes.h>
#include "pluginlib/class_list_macros.h"
#include "20_sim_interaction_control/common/xxmatrix.h"

//#define DEBUG

PLUGINLIB_DECLARE_CLASS(youbot_description, CartesianInteractionController, controller::CartesianInteractionController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

    /*
    static tf::TransformBroadcaster br;

    void publishTf(double* tf, string parent, string child) {
        tf::Transform trans;
        trans.setOrigin(tf::Vector3(tf[3],tf[7],tf[11]));
        btMatrix3x3 rotMatrix(tf[0],tf[1],tf[2],
                              tf[4],tf[5],tf[6],
                              tf[8],tf[9],tf[10]);
        tf::Quaternion quat;
        rotMatrix.getRotation(quat);
        trans.setRotation(quat);
        br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),parent,child));

    }
*/

    CartesianInteractionController* CartesianInteractionControllerDebug::getControllerPtr() {
        return dynamic_cast<CartesianInteractionController*>(this->controllerPtr);
    }

    void CartesianInteractionControllerDebug::init() {
         // Calculating time interval dt between cycles
 /*       using namespace boost::units;
        vector <brics_actuator::JointValue> positionsGazebo;
        vector <brics_actuator::JointValue> positionsController;
        vector <brics_actuator::JointValue> torquesGazebo;
*/
        ros::NodeHandle &nodeHandle = getControllerPtr()->nodeHandle;

        gazeboJointPositions.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointPositions > (nodeHandle, "gazebo_joint_positions", 1));
        gazeboJointPositions->lock();

        controllerJointPositions.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointPositions > (nodeHandle, "controller_joint_positions", 1));
        controllerJointPositions->lock();

        gazeboJointTorques.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointTorques> (nodeHandle, "gazebo_joint_torques", 1));
        gazeboJointTorques->lock();

        controllerJointTorques.reset(new realtime_tools::RealtimePublisher<brics_actuator::JointTorques> (nodeHandle, "controller_joint_torques", 1));
        controllerJointTorques->lock();

        gazeboJointPose.reset(new realtime_tools::RealtimePublisher<brics_actuator::CartesianPose> (nodeHandle, "gazebo_joint_pose", 1));
        gazeboJointPose->lock();

        controllerJointPose.reset(new realtime_tools::RealtimePublisher<brics_actuator::CartesianPose> (nodeHandle, "controller_joint_pose", 1));
        controllerJointPose->lock();

    }

    void CartesianInteractionControllerDebug::publish() {

        vector <brics_actuator::JointValue> positionsGazebo;
        vector <brics_actuator::JointValue> positionsController;
        vector <brics_actuator::JointValue> torquesGazebo;

        /* std::vector<pr2_mechanism_model::JointState*>* joints = &(getControllerPtr()->joints);

         gazeboJointPositions->trylock();
         controllerJointPositions->trylock();
         gazeboJointTorques->trylock();

         for (unsigned int j = 0; j < joints->size(); ++j) {

             positionsGazebo[j].joint_uri = joints->joint_->name;
             positionsGazebo[j].value = joints[j]->position_;
             positionsGazebo[j].unit = to_string(si::radians);
             positionsGazebo[j].timeStamp = currentTime;

             positionsController[j].joint_uri = joints[j]->joint_->name;
             positionsController[j].value = u[28+j]; //CHANGE THAT!
             positionsController[j].unit = to_string(si::radians);
             positionsController[j].timeStamp = currentTime;

             torquesGazebo[j].joint_uri = joints[j]->joint_->name;
             torquesGazebo[j].value = joints[j]->commanded_effort_;
             torquesGazebo[j].unit = to_string(si::newton_meter);
             torquesGazebo[j].timeStamp = currentTime;
       }

                gazeboJointPositions->msg_.positions = positionsGazebo;
                controllerJointPositions->msg_.positions = positionsController;
                gazeboJointTorques->msg_.torques = torquesGazebo;

                controllerStatePublisher->unlockAndPublish();
                gazeboJointPositions->unlockAndPublish();
                controllerJointPositions->unlockAndPublish();
                gazeboJointTorques->unlockAndPublish();


/*

                publishTf((y+5),"/base_link", "/Htip0");

                XXMatrix* matrix = my20simSubmodel.m_M;
                double* joint1 = matrix[33].mat;
                publishTf(joint1,"/base_link", "/joint1");
                double* joint2 = matrix[34].mat;
                publishTf(joint2,"/base_link", "/joint2");
                double* joint3 = matrix[35].mat;
                publishTf(joint3,"/base_link", "/joint3");
                double* joint4 = matrix[36].mat;
                publishTf(joint4,"/base_link", "/joint4");
                double* joint5 = matrix[37].mat;
                publishTf(joint5,"/base_link", "/joint5");

                /*        tf::Transform trans;
        trans.setOrigin(tf::Vector3(wrench.force.x,wrench.force.y,wrench.force.z));
        tf::Quaternion quat = tf::createQuaternionFromRPY(wrench.torque.x, wrench.torque.y, wrench.torque.z);
        trans.setRotation(quat);
        br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/base_link","/target"));
*/



    }


    CartesianInteractionController::CartesianInteractionController()
    : robotPtr(NULL) {
        loopCount = 0;
#ifdef DEBUG
        debugInfo = new CartesianInteractionControllerDebug(this);
#else
        debugInfo = new Debug(this);
#endif

    }

    CartesianInteractionController::~CartesianInteractionController() {
        subscriber.shutdown();
        autoGenerated20simController.Terminate(u, y);
        delete debugInfo;
    }

/*  auto-generated by 20sim;
    initialize the inputs and outputs with correct initial values */
    void CartesianInteractionController::init20SimController() {

        u[0] = 0.0; /* joints.f */
        u[1] = 0.0;
        u[2] = 0.0;
        u[3] = 0.0;
        u[4] = 0.0;

        u[5] = 0.024;
        u[6] = 0.033;
        u[7] = 0.0;
        u[8] = 0.0;
        u[9] = 0.0;
        u[10] = 0.0;

        u[11] = 0.0;
        u[12] = 0.0;
        u[13] = 0.0;
        u[14] = 0.0;
        u[15] = 0.0;
        u[16] = 0.0;

        u[17] = 0.096;
        u[18] = 0.019;
        u[19] = 0.155;
        u[20] = 0.135;
        u[21] = 0.096;
        u[22] = 0.034;

        u[23] = 0.0;
        u[24] = 0.0;
        u[25] = 0.0;
        u[26] = 0.0;
        u[27] = 0.0;

        u[28] = 0.0; /* tip.e */
        u[29] = 0.0;
        u[30] = 0.0;
        u[31] = 0.0;
        u[32] = 0.0;
        u[33] = 0.0;

        y[0] = 0.0; /* Htip0 */
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
        y[16] = 0.0; /* joints.e */
        y[17] = 0.0;
        y[18] = 0.0;
        y[19] = 0.0;
        y[20] = 0.0;

        autoGenerated20simController.Initialize(u, y, 0.0);

    }

    bool CartesianInteractionController::init(pr2_mechanism_model::RobotState *robotPtr, ros::NodeHandle &nodeHandle) {
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
        subscriber = nodeHandle.subscribe("command", 1, &CartesianInteractionController::positionCommand, this);

        // Initializing 20Sim controller
        init20SimController();

        // Initializing debug output
        debugInfo->init();

        return true;
    }

    void CartesianInteractionController::starting() {
         // Initializing timer
        lastTime = robotPtr->getTime();
    }

    void CartesianInteractionController::udpate20SimControl() {

        // setting current joint velocities to the 20Sim controller
        u[0] = joints[0]->velocity_; /* joints.f */
        u[1] = joints[1]->velocity_;
        u[2] = joints[2]->velocity_;
        u[3] = joints[3]->velocity_;
        u[4] = joints[4]->velocity_;

        // setting current joint positions + offset to the 20Sim controller
        u[23] = -joints[0]->position_ + 170 * M_PI / 180; /* q */
        u[24] = joints[1]->position_ - 65 * M_PI / 180;
        u[25] = joints[2]->position_ + 146 * M_PI / 180;
        u[26] = joints[3]->position_ - 102.5 * M_PI / 180;
        u[27] = joints[4]->position_ + 167.5 * M_PI / 180;

        // setting target pose for the end effector
        u[28] = position[0];
        u[29] = position[1];
        u[30] = position[2];
        u[31] = orientation[0];
        u[32] = orientation[0];
        u[33] = orientation[0];

        // running 20Sim controller
        autoGenerated20simController.Calculate(u, y);

        targetEfforts[0] = y[0]; /* joints.e */
        targetEfforts[1] = y[1];
        targetEfforts[2] = y[2];
        targetEfforts[3] = y[3];
        targetEfforts[4] = y[4];


    }

    void CartesianInteractionController::update() {

        ros::Time currentTime = robotPtr->getTime();
        ros::Duration dt = currentTime - lastTime;
        lastTime = currentTime;

        // Initializing error vector
        udpate20SimControl();
        // Doing control here, calculating and applying the efforts
        for (unsigned int i = 0; i < joints.size(); ++i) {
            joints[i]->commanded_effort_ = targetEfforts[i];
        }

        // if debug mode is set, publishing debug TFs and controller information
        if (loopCount % 10 == 0) {
            debugInfo->publish();
        }
        ++loopCount;
    }



    void CartesianInteractionController::positionCommand(const brics_actuator::CartesianPose &pose) {

        using namespace boost::units;

        brics_actuator::CartesianVector tipPosition;
		tipPosition = pose.position;

		tf::Quaternion tipOrientation;
		tf::quaternionMsgToTF(pose.orientation, tipOrientation);

		if (tipPosition.unit != to_string(si::meter))
            ROS_ERROR("Position value is set in the inpcompatible units %s, expecting meters", tipPosition.unit.c_str());

        this->position[0] = tipPosition.x;
		this->position[1] = tipPosition.y;
		this->position[2] = tipPosition.z;

        btMatrix3x3(tipOrientation).getRPY( this->orientation[0], this->orientation[1], this->orientation[2]);
    }

}

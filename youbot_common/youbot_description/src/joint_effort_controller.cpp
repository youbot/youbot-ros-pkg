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
#include <boost/units/systems/si/torque.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <tf/transform_broadcaster.h>
#include <boost/units/io.hpp>
#include "youbot_controllers/joint_effort_controller.h"
#include "pluginlib/class_list_macros.h"
#include "../src/position_interaction_control/common/xxmatrix.h"




PLUGINLIB_DECLARE_CLASS(youbot_description, JointEffortController, controller::JointEffortController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

    JointEffortController::JointEffortController()
    : robotPtr(NULL) {
        loopCount = 0;

    }

    JointEffortController::~JointEffortController() {
        subscriber.shutdown();
        my20simSubmodel.Terminate(u, y);
    }

    /* when a controller gets initialized, the controller manager passes the controller a pointer to the RobotState
       RobotState is an interface to the robot joints and a description of the robot model
       for details see http://www.ros.org/wiki/pr2_mechanism_model */

    void JointEffortController::init20SimController() {

        /* initialize the inputs and outputs with correct initial values */
        u[0] = 0.0; /* joints.f */
        u[1] = 0.0;
        u[2] = 0.0;
        u[3] = 0.0;
        u[4] = 0.0;

        u[5] = 0.024;
        u[6] = 0.033; //0.0;
        u[7] = 0.0;
        u[8] = 0.0;
        u[9] = 0;
        u[10] = 0;

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





       /* u[5] = 0.024;
        u[6] = 0.0; //0.0;
        u[7] = 0.096;

        u[8] = 0.033;
        u[9] = 0.0;
        u[10] = 0.019;

        u[11] = 0.0;
        u[12] = 0.0;
        u[13] = 0.115;

        u[14] = 0.0;
        u[15] = 0.0;
        u[16] = 0.135;

        u[17] = 0.0;
        u[18] = -0.010;
        u[19] = 0.029;

        u[20] = 0.0002;
        u[21] = 0.0;
        u[22] = 0.130;
*/
        u[23] = 0; //170 * M_PI / 180; //2.9671 /* q */
        u[24] = 0; //65 * M_PI / 180; //-1.1345
        u[25] = 0; //-146 * M_PI / 180; //2.5482
        u[26] = 0; //102.5 * M_PI / 180; //-1.7890
        u[27] = 0; //167.5 * M_PI / 180; //2.9234

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

        my20simSubmodel.Initialize(u, y, 0.0);

    }

    bool JointEffortController::init(pr2_mechanism_model::RobotState *robotPtr, ros::NodeHandle &nodeHandle) {
        using namespace XmlRpc;
        this->nodeHandle = nodeHandle;
        this->robotPtr = robotPtr;

        ROS_INFO("Initializing effort control for the youbot arm...\n");

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

        // Initializing targetefforts vector
        targetEfforts.resize(joints.size());

        // Sets up pid controllers for all of the joints from yaml file
        std::string gainsNS;

        if (!nodeHandle.getParam("gains", gainsNS))
            gainsNS = nodeHandle.getNamespace() + "/gains";

        pids.resize(joints.size());

        for (unsigned int i = 0; i < joints.size(); ++i) {
            if (!pids[i].init(ros::NodeHandle(gainsNS + "/" + joints[i]->joint_->name))) {
                ROS_ERROR("Can't setup PID for the joint %s. (namespace: %s)", joints[i]->joint_->name.c_str(), nodeHandle.getNamespace().c_str());
                return false;
            }

            double p, i, d, i_max, i_min;
            pids[i].getGains(p, i, d, i_max, i_min);
            ROS_DEBUG("PID for joint %s: p=%f, i=%f, d=%f, i_max=%f, i_min=%f\n", joints[i]->joint_->name.c_str(), p, i, d, i_max, i_min);
        }

        subscriber = nodeHandle.subscribe("command", 1, /*&JointEffortController::velocityCommand*/ &JointEffortController::wrenchCommand, this);

        controllerStatePublisher.reset(new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointTrajectoryControllerState > (nodeHandle, "state", 1));
        controllerStatePublisher->lock();

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

	    for (size_t j = 0; j < joints.size(); ++j) {
            controllerStatePublisher->msg_.joint_names.push_back(joints[j]->joint_->name);

        }


        controllerStatePublisher->msg_.desired.positions.resize(joints.size());
        controllerStatePublisher->msg_.desired.velocities.resize(joints.size());
        controllerStatePublisher->msg_.desired.accelerations.resize(joints.size());
        controllerStatePublisher->msg_.actual.positions.resize(joints.size());
        controllerStatePublisher->msg_.actual.velocities.resize(joints.size());
        controllerStatePublisher->msg_.error.positions.resize(joints.size());
        controllerStatePublisher->msg_.error.velocities.resize(joints.size());
        controllerStatePublisher->unlock();


        init20SimController();
        return true;
    }

    void JointEffortController::starting() {
        ROS_DEBUG("Starting effort controls for the joints\n");

        for (unsigned int i = 0; i < pids.size(); ++i)
            pids[i].reset();

        // Initializing timer
        lastTime = robotPtr->getTime();
    }

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


    void JointEffortController::udpate20SimControl(brics_actuator::CartesianWrench &wrench) {
        u[0] = joints[0]->velocity_; /* joints.f */
        u[1] = joints[1]->velocity_;
        u[2] = joints[2]->velocity_;
        u[3] = joints[3]->velocity_;
        u[4] = joints[4]->velocity_;
        // u

        u[23] = -joints[0]->position_ + 170 * M_PI / 180; /* q */
        u[24] = joints[1]->position_ - 65 * M_PI / 180;
        u[25] = joints[2]->position_ + 146 * M_PI / 180;
        u[26] = joints[3]->position_ - 102.5 * M_PI / 180;
        u[27] = joints[4]->position_ + 167.5 * M_PI / 180;

        u[28] = wrench.force.x; /* tip.e */
        u[29] = wrench.force.y;
        u[30] = wrench.force.z;
        u[31] = wrench.torque.x;
        u[32] = wrench.torque.y;
        u[33] = wrench.torque.z;


        tf::Transform trans;
        trans.setOrigin(tf::Vector3(wrench.force.x,wrench.force.y,wrench.force.z));
        tf::Quaternion quat = tf::createQuaternionFromRPY(wrench.torque.x, wrench.torque.y, wrench.torque.z);
        trans.setRotation(quat);
        br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/base_link","/target"));

        my20simSubmodel.Calculate(u, y);
        //y[0] = 0.0;		/* Htip0 */
        /*y[1] = 0.0;
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
        y[15] = 0.0;*/
        targetEfforts[0] = y[0]; /* joints.e */
        targetEfforts[1] = y[1];
        targetEfforts[2] = y[2];
        targetEfforts[3] = y[3];
        targetEfforts[4] = y[4];
        //ROS_INFO("Effort: %f, %f, %f, %f, %f \n", targetEfforts[0], targetEfforts[1], targetEfforts[2], targetEfforts[3], targetEfforts[4]);
        //ROS_INFO("Position: %f, %f, %f, %f, %f, %f \n", wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z);
        //y[21] = 0.0;		/* tip.f */
        /*y[22] = 0.0;
        y[23] = 0.0;
        y[24] = 0.0;
        y[25] = 0.0;
        y[26] = 0.0;*/
    }

    void JointEffortController::publish() {




    }



    void JointEffortController::update() {
        // Calculating time interval dt between cycles
        using namespace boost::units;
        vector <brics_actuator::JointValue> positionsGazebo;
        vector <brics_actuator::JointValue> positionsController;
        vector <brics_actuator::JointValue> torquesGazebo;

        ros::Time currentTime = robotPtr->getTime();
        ros::Duration dt = currentTime - lastTime;
        lastTime = currentTime;
        //ROS_INFO("Controller\n");
        // Initializing error vector
        std::vector<double> error(joints.size());
        udpate20SimControl(this->wrench);
        // Doing control here, calculating and applying the efforts
        for (unsigned int i = 0; i < joints.size(); ++i) {
            joints[i]->commanded_effort_ = targetEfforts[i];

        }

        if (loopCount % 10 == 0) {
            positionsGazebo.resize(joints.size());
            positionsController.resize(joints.size());;
            torquesGazebo.resize(joints.size());

            //if (controllerStatePublisher && controllerStatePublisher->trylock()) {

            controllerStatePublisher->trylock();
            gazeboJointPositions->trylock();
            controllerJointPositions->trylock();
            gazeboJointTorques->trylock();


                controllerStatePublisher->msg_.header.stamp = currentTime;
                for (unsigned int j = 0; j < joints.size(); ++j) {
                    controllerStatePublisher->msg_.desired.positions[j] = 0;
                    controllerStatePublisher->msg_.desired.velocities[j] = 0;
                    controllerStatePublisher->msg_.desired.accelerations[j] = 0;
                    controllerStatePublisher->msg_.actual.positions[j] = joints[j]->position_;
                    controllerStatePublisher->msg_.actual.velocities[j] = joints[j]->velocity_;
                    controllerStatePublisher->msg_.error.positions[j] = 0;
                    controllerStatePublisher->msg_.error.velocities[j] = 0;

                    positionsGazebo[j].joint_uri = joints[j]->joint_->name;
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





 //y[0] = 0.0;		/* Htip0 */
        /*y[1] = 0.0;
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
        y[15] = 0.0;*/


              /*  for (int i = 0; i < 21; i++) {
                    ROS_INFO("y[%d]=%f ,",i,y[i]);

                }
                ROS_INFO("\n\n");
*/


     /*           trans.setOrigin(tf::Vector3(y[8],y[12],y[16]));

                btMatrix3x3 rotMatrix(y[5],y[6],y[7],
                                      y[9],y[10],y[11],
                                      y[13],y[14],y[15]);
                tf::Quaternion quat;
                rotMatrix.getRotation(quat);
                trans.setRotation(quat);
                br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"/base_link","/Htip0"));
*/
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

            //}
        }




        ++loopCount;
        //	ROS_INFO("%f\n", joints[1]->commanded_effort_);
    }
    //TODO refacor the name of the function

    void JointEffortController::velocityCommand(const brics_actuator::JointTorques &jointEfforts) {
        ROS_DEBUG("Readin the target effort from brics_actuator::jointEfforts message\n");
        std::vector <brics_actuator::JointValue> efforts = jointEfforts.torques;

        if (efforts.empty()) {
            starting();
            return;
        }

        //Correlates the joints we're commanding to the joints in the message
        std::vector<int> lookup(joints.size(), -1); // Maps from an index in joints_ to an index in the msg
        for (unsigned int j = 0; j < joints.size(); ++j) {

            for (unsigned int k = 0; k < efforts.size(); ++k) {
                if (efforts[k].joint_uri == joints[j]->joint_->name) {
                    lookup[j] = k;
                    break;
                }
            }
            if (lookup[j] == -1)
                ROS_ERROR("Unable to locate joint %s in the commanded efforts.", joints[j]->joint_->name.c_str());
        }

        std::vector<double> actualefforts;
        targetEfforts.resize(efforts.size());
        using namespace boost::units;

        for (unsigned int j = 0; j < joints.size(); ++j) {
            if (lookup[j] != -1) {
                ROS_DEBUG("Joint %s = %f %s, ", efforts[lookup[j]].joint_uri.c_str(), efforts[lookup[j]].value, efforts[lookup[j]].unit.c_str());
                if (efforts[lookup[j]].unit != to_string(si::newton_meter))
                    ROS_ERROR("Joint %s has the value in the inpcompatible units %s", efforts[lookup[j]].joint_uri.c_str(), efforts[lookup[j]].unit.c_str());
                if (!targetEfforts.empty())
                    targetEfforts[j] = efforts[lookup[j]].value;
            }
        }
    }

    void JointEffortController::wrenchCommand(const brics_actuator::CartesianWrench &wrench) {
        ROS_INFO("Readin the target wrench from brics_actuator::CartesianWrench message\n");
        this->wrench = wrench;
    }
}

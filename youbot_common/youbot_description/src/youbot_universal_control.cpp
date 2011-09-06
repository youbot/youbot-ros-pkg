/******************************************************************************
 * Copyright (c) 2011
 * Locomotec, University of Twente
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
 * * Neither the name of Locomotec and University of Twente nor the names of its
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
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/io.hpp>
#include "youbot_universal_control.h"
#include "pluginlib/class_list_macros.h"
#include "angles/angles.h"

PLUGINLIB_DECLARE_CLASS(youbot_description, YouBotUniversalController, controller::YouBotUniversalController, pr2_controller_interface::Controller)

namespace controller
{

YouBotUniversalController::YouBotUniversalController()
    : currentControlMode(POSITION), robotPtr(NULL)
{

}

YouBotUniversalController::~YouBotUniversalController()
{
    positionCommandSubscriber.shutdown();
    velocityCommandSubscriber.shutdown();
    torqueCommandSubscriber.shutdown();
}

/* when a controller gets initialized, the controller manager passes the controller a pointer to the RobotState
   RobotState is an interface to the robot joints and a description of the robot model
   for details see http://www.ros.org/wiki/pr2_mechanism_model */

bool YouBotUniversalController::init(pr2_mechanism_model::RobotState *robotPtr, ros::NodeHandle &nodeHandle)
{
    using namespace XmlRpc;
    this->nodeHandle = nodeHandle;
    this->robotPtr = robotPtr;



    ROS_INFO("Initializing joint position control...\n");

    // Gets all of the joint pointers from the RobotState to a joints vector
    XmlRpc::XmlRpcValue jointNames;
    if (!nodeHandle.getParam("joints", jointNames))
    {
        ROS_ERROR("No joints given. (namespace: %s)", nodeHandle.getNamespace().c_str());
        return false;
    }

    if (jointNames.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Malformed joint specification.  (namespace: %s)", nodeHandle.getNamespace().c_str());
        return false;
    }

    for (unsigned int i = 0; i < static_cast<unsigned int> (jointNames.size()); ++i)
    {
        XmlRpcValue &name = jointNames[i];
        if (name.getType() != XmlRpcValue::TypeString)
        {
            ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)", nodeHandle.getNamespace().c_str());
            return false;
        }

        pr2_mechanism_model::JointState *jointStatePtr = robotPtr->getJointState((std::string)name);
        if (jointStatePtr == NULL)
        {
            ROS_ERROR("Joint not found: %s. (namespace: %s)", ((std::string)name).c_str(), nodeHandle.getNamespace().c_str());
            return false;
        }

        joints.push_back(jointStatePtr);
    }

    // Ensures that all the joints are calibrated.
    for (unsigned int i = 0; i < joints.size(); ++i)
    {
        if (!joints[i]->calibrated_)
        {
            ROS_ERROR("Joint %s was not calibrated (namespace: %s)", joints[i]->joint_->name.c_str(), nodeHandle.getNamespace().c_str());
            return false;
        }
    }

    // Initializing targetVelocities vector
    setPoints.resize(joints.size());

    // Sets up pid controllers for all of the joints from yaml file
    std::string gainsNS;

    if (!nodeHandle.getParam("gains", gainsNS))
        gainsNS = nodeHandle.getNamespace() + "/gains";

    pids.resize(joints.size());

    for (unsigned int i = 0; i < joints.size(); ++i)
    {
        if (!pids[i].init(ros::NodeHandle(gainsNS + "/" + joints[i]->joint_->name)))
        {
            ROS_ERROR("Can't setup PID for the joint %s. (namespace: %s)", joints[i]->joint_->name.c_str(), nodeHandle.getNamespace().c_str());
            return false;
        }

        double p, i, d, i_max, i_min;
        pids[i].getGains(p, i, d, i_max, i_min);
        ROS_DEBUG("PID for joint %s: p=%f, i=%f, d=%f, i_max=%f, i_min=%f\n", joints[i]->joint_->name.c_str(), p, i, d, i_max, i_min);
    }

    positionCommandSubscriber = nodeHandle.subscribe("position_command", 1, &YouBotUniversalController::positionCommand, this);
    velocityCommandSubscriber = nodeHandle.subscribe("velocity_command", 1, &YouBotUniversalController::velocityCommand, this);
    torqueCommandSubscriber = nodeHandle.subscribe("torque_command", 1, &YouBotUniversalController::torqueCommand, this);
    return true;
}

void YouBotUniversalController::starting()
{
    ROS_DEBUG("Starting velocity controls for the joints\n");

    for (unsigned int i = 0; i < pids.size(); ++i)
        pids[i].reset();

    // Initializing timer
    lastTime = robotPtr->getTime();
}

void YouBotUniversalController::update()
{

    ros::Time currentTime = robotPtr->getTime();
    ros::Duration dt = currentTime - lastTime;
    lastTime = currentTime;

    for (unsigned int i = 0; i < joints.size(); i++)
    {
        switch (currentControlMode)
        {
        case YouBotUniversalController::POSITION :
        {
            updateJointPosition(setPoints[i], joints[i], &pids[i], dt);
            break;
        }
        case YouBotUniversalController::VELOCITY :
        {
            updateJointVelocity(setPoints[i], joints[i], &pids[i], dt);
            break;
        }
        case YouBotUniversalController::TORQUE :
        {
            updateJointTorque(setPoints[i], joints[i], &pids[i], dt);
            break;
        }
        default :
        {
            ROS_WARN("Joint %d has unsupported control mode \n", i);
        }

        }

    }

}

void YouBotUniversalController::updateJointVelocity(double setPoint, pr2_mechanism_model::JointState* joint_state_, control_toolbox::Pid* pid_controller_, const ros::Duration& dt)
{
    double error(0);
    assert(joint_state_->joint_);

    double command_ = setPoint;
    double velocity_ = joint_state_->velocity_;
    const double T = 1;
    filteredVelocity = filteredVelocity + (velocity_ - filteredVelocity) * dt.toSec()/(dt.toSec()+T);
    error = filteredVelocity - command_;
    ROS_DEBUG("Current velocity: %f, filtered velocity: %f, Error: %f\n",velocity_, filteredVelocity, error);

    double commanded_effort = pid_controller_ -> updatePid(error, dt);
    joint_state_->commanded_effort_ = commanded_effort;

}

void YouBotUniversalController::updateJointTorque(double setPoint, pr2_mechanism_model::JointState* joint_state_, control_toolbox::Pid* pid_controller_, const ros::Duration& dt)
{
    assert(joint_state_->joint_);
    joint_state_->commanded_effort_ = setPoint;
}

void YouBotUniversalController::updateJointPosition(double setPoint, pr2_mechanism_model::JointState* joint_state_, control_toolbox::Pid* pid_controller_, const ros::Duration& dt)
{
    double error(0);

    assert(joint_state_->joint_);

    double command_ = setPoint;
    ROS_DEBUG("Current position: %f, target position: %f", joint_state_->position_, command_);
    if(joint_state_->joint_->type == urdf::Joint::REVOLUTE)
    {
        angles::shortest_angular_distance_with_limits(command_, joint_state_->position_, joint_state_->joint_->limits->lower, joint_state_->joint_->limits->upper,error);

    }
    else if(joint_state_->joint_->type == urdf::Joint::CONTINUOUS)
    {
        error = angles::shortest_angular_distance(command_, joint_state_->position_);
    }
    else if(joint_state_->joint_->type == urdf::Joint::PRISMATIC)//prismatic
    {
        error = joint_state_->position_ - command_;
    }
    else
    {
        ROS_WARN("Joint %s has unsupported type \n",joint_state_->joint_->name.c_str());
        error = joint_state_->position_ - command_;
    }

    //double commanded_effort = pid_controller_.updatePid(error, dt_);
    double commanded_effort = pid_controller_ -> updatePid(error, dt); // assuming desired velocity is 0
    joint_state_->commanded_effort_ = commanded_effort;
}

void YouBotUniversalController::torqueCommand(const brics_actuator::JointTorques &jointTorques)
{
    currentControlMode = YouBotUniversalController::TORQUE;

    ROS_DEBUG("Readin the target torques from the brics_actuator::JointVelocities message\n");
    std::vector <brics_actuator::JointValue> torques = jointTorques.torques;

    if (torques.empty())
    {
        starting();
        return;
    }

    //Correlates the joints we're commanding to the joints in the message
    std::vector<int> lookup(joints.size(), -1); // Maps from an index in joints_ to an index in the msg
    for (unsigned int j = 0; j < joints.size(); ++j)
    {

        for (unsigned int k = 0; k < torques .size(); ++k)
        {
            if (torques[k].joint_uri == joints[j]->joint_->name)
            {
                lookup[j] = k;
                break;
            }
        }
        if (lookup[j] == -1)
            ROS_ERROR("Unable to locate joint %s in the commanded torques.", joints[j]->joint_->name.c_str());
    }

    std::vector<double> actualTorques;
    setPoints.resize(torques.size());
    using namespace boost::units;

    for (unsigned int j = 0; j < joints.size(); ++j)
    {
        if (lookup[j] != -1)
        {
            ROS_DEBUG("Joint %s = %f %s, ", torques[lookup[j]].joint_uri.c_str(), torques[lookup[j]].value, torques[lookup[j]].unit.c_str());
            if (torques[lookup[j]].unit != to_string(si::newton_meter))
                ROS_ERROR("Joint %s has the value in the inpcompatible units %s", torques[lookup[j]].joint_uri.c_str(), torques[lookup[j]].unit.c_str());
            if (!setPoints.empty())
                setPoints[j] = torques[lookup[j]].value;
        }
    }


}

void YouBotUniversalController::velocityCommand(const brics_actuator::JointVelocities &jointVelocities)
{
    currentControlMode = YouBotUniversalController::VELOCITY;

    ROS_DEBUG("Readin the target velocities from the brics_actuator::JointVelocities message\n");
    std::vector <brics_actuator::JointValue> velocities = jointVelocities.velocities;

    if (velocities.empty())
    {
        starting();
        return;
    }

    //Correlates the joints we're commanding to the joints in the message
    std::vector<int> lookup(joints.size(), -1); // Maps from an index in joints_ to an index in the msg
    for (unsigned int j = 0; j < joints.size(); ++j)
    {

        for (unsigned int k = 0; k < velocities.size(); ++k)
        {
            if (velocities[k].joint_uri == joints[j]->joint_->name)
            {
                lookup[j] = k;
                break;
            }
        }
        if (lookup[j] == -1)
            ROS_ERROR("Unable to locate joint %s in the commanded velocities.", joints[j]->joint_->name.c_str());
    }

    std::vector<double> actualVelocities;
    setPoints.resize(velocities.size());
    using namespace boost::units;

    for (unsigned int j = 0; j < joints.size(); ++j)
    {
        if (lookup[j] != -1)
        {
            ROS_DEBUG("Joint %s = %f %s, ", velocities[lookup[j]].joint_uri.c_str(), velocities[lookup[j]].value, velocities[lookup[j]].unit.c_str());
            if (velocities[lookup[j]].unit != to_string(si::radian_per_second))
                ROS_ERROR("Joint %s has the value in the inpcompatible units %s", velocities[lookup[j]].joint_uri.c_str(), velocities[lookup[j]].unit.c_str());
            if (!setPoints.empty())
                setPoints[j] = velocities[lookup[j]].value;
        }
    }

}

void YouBotUniversalController::positionCommand(const brics_actuator::JointPositions &jointPositions)
{
    currentControlMode = YouBotUniversalController::POSITION;

    ROS_DEBUG("Readin the target positions from the brics_actuator::JointPositions message\n");
    std::vector <brics_actuator::JointValue> positions = jointPositions.positions;

    if (positions.empty())
    {
        starting();
        return;
    }

    //Correlates the joints we're commanding to the joints in the message
    std::vector<int> lookup(joints.size(), -1); // Maps from an index in joints to an index in the msg
    for (unsigned int j = 0; j < joints.size(); ++j)
    {
        for (unsigned int k = 0; k < positions.size(); ++k)
        {
            if (positions[k].joint_uri == joints[j]->joint_->name)
            {
                lookup[j] = k;
                break;
            }
        }
        if (lookup[j] == -1)
            ROS_ERROR("Unable to locate joint %s in the commanded message.", joints[j]->joint_->name.c_str());
    }

    std::vector<double> actualPositions;
    setPoints.resize(positions.size());
    using namespace boost::units;

    for (unsigned int j = 0; j < joints.size(); ++j)
    {
        if (lookup[j] != -1)
        {
            ROS_DEBUG("Joint %s = %f %s, ", positions[lookup[j]].joint_uri.c_str(), positions[lookup[j]].value, positions[lookup[j]].unit.c_str());
            if (joints[j]->joint_->type == urdf::Joint::REVOLUTE)
            {
                if (positions[lookup[j]].unit != to_string(si::radian))
                    ROS_ERROR("Joint %s has a value in the inpcompatible units %s, expecting %s", positions[lookup[j]].joint_uri.c_str(), positions[lookup[j]].unit.c_str(), to_string(si::radian).c_str());
            }
            else if (joints[j]->joint_->type == urdf::Joint::PRISMATIC)
            {
                if (positions[lookup[j]].unit != to_string(si::meter))
                    ROS_ERROR("Joint %s has a value in the inpcompatible units %s, expecting %s", positions[lookup[j]].joint_uri.c_str(), positions[lookup[j]].unit.c_str(), to_string(si::meter).c_str());

            }

            if (!setPoints.empty())
                setPoints[j] = positions[lookup[j]].value;
        }
    }
}
} // end of the namespace


/******************************************************************************
 * Copyright (c) 2011
 * GPS GmbH
 *
 * Author:
 * Alexey Zakharov
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
 * * Neither the name of GPS GmbH nor the names of its
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


#ifndef JOINTTRAJECTORYACTION_H
#define	JOINTTRAJECTORYACTION_H

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

namespace KDL
{
class Trajectory_Composite;
}

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class JointStateObserver;

class JointTrajectoryAction
{
public:
    JointTrajectoryAction(JointStateObserver* jointStateObserver);
    JointTrajectoryAction(JointStateObserver* youbot,
                          double velocityGain, double positionGain, double frequency);
    JointTrajectoryAction(const JointTrajectoryAction& orig);
    virtual ~JointTrajectoryAction();

    void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as);

    void jointStateCallback(const sensor_msgs::JointState& joint_state);
    void jointStateCallback(const brics_actuator::JointPositions& position,
                            const brics_actuator::JointVelocities& velocity);

    void setVelocityGain(double velocityGain);
    double getVelocityGain() const;


    void setPositionGain(double positionGain);
    double getPositionGain() const;

    void setFrequency(double frequency);
    double getFrequency() const;



private:

    double velocityGain;
    double positionGain;
    double frequency;

    sensor_msgs::JointState current_state;
    JointStateObserver* jointStateObserver;

private:

    double calculateVelocity(double actualAngle,
                             double actualVelocity,
                             const KDL::Trajectory_Composite& trajectoryComposite,
                             double elapsedTimeInSec);

    void controlLoop(const std::vector<double>& actualJointAngles,
                     const std::vector<double>& actualJointVelocities,
                     const KDL::Trajectory_Composite* trajectoryComposite,
                     int numberOfJoints,
                     ros::Time startTime,
                     std::vector<double>& velocities);

    void setTargetTrajectory(double angle1,
                             double angle2,
                             double duration,
                             KDL::Trajectory_Composite& trajectoryComposite);

};

#endif	/* JOINTTRAJECTORYACTION_H */


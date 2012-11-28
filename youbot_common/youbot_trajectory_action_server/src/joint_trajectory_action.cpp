
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

#include <youbot_trajectory_action_server/joint_trajectory_action.h>
#include <youbot_trajectory_action_server/joint_state_observer.h>

#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/angular_velocity.hpp>


JointTrajectoryAction::JointTrajectoryAction(JointStateObserver* jointStateObserver) : jointStateObserver(jointStateObserver)
{


    setPositionGain(5.0);
    setVelocityGain(0.0);
    setFrequency(50);

}

JointTrajectoryAction::JointTrajectoryAction(JointStateObserver* jointStateObserver,
                                             double positionGain,
                                             double velocityGain,
                                             double frequency) : jointStateObserver(jointStateObserver)
{


    setFrequency(frequency);
    setPositionGain(positionGain);
    setVelocityGain(velocityGain);

}

JointTrajectoryAction::JointTrajectoryAction(const JointTrajectoryAction& orig) : jointStateObserver(orig.jointStateObserver)
{

    setPositionGain(orig.getPositionGain());
    setVelocityGain(orig.getVelocityGain());
    setFrequency(orig.getFrequency());
}

JointTrajectoryAction::~JointTrajectoryAction()
{

}

void JointTrajectoryAction::setFrequency(double frequency)
{
    this->frequency = frequency;
}

double JointTrajectoryAction::getFrequency() const
{
    return frequency;
}

void JointTrajectoryAction::setVelocityGain(double velocityGain)
{
    this->velocityGain = velocityGain;
}

double JointTrajectoryAction::getVelocityGain() const
{
    return velocityGain;
}

void JointTrajectoryAction::setPositionGain(double positionGain)
{
    this->positionGain = positionGain;
}

double JointTrajectoryAction::getPositionGain() const
{
    return positionGain;
}

double JointTrajectoryAction::calculateVelocity(double actualAngle,
                                                double actualVelocity,
                                                const KDL::Trajectory_Composite& trajectoryComposite,
                                                double elapsedTimeInSec)
{

    double error = 0;

    if (trajectoryComposite.Duration() > 0 && elapsedTimeInSec <= trajectoryComposite.Duration())
    {
        double actualTime = elapsedTimeInSec;

        double desiredAngle = trajectoryComposite.Pos(actualTime).p.x();
        double desiredVelocity = trajectoryComposite.Vel(actualTime).vel.x();
        double velocityError = desiredVelocity - actualVelocity;
        double positionError = desiredAngle - actualAngle;
        double gain1 = getPositionGain();
        double gain2 = getVelocityGain();

        error = gain1 * positionError + gain2 * velocityError;

    }

    return error;
}

void JointTrajectoryAction::controlLoop(const std::vector<double>& actualJointAngles,
                                        const std::vector<double>& actualJointVelocities,
                                        const KDL::Trajectory_Composite* trajectoryComposite,
                                        int numberOfJoints,
                                        ros::Time startTime,
                                        std::vector<double>& velocities)
{

    velocities.clear();
    double elapsedTime = ros::Duration(ros::Time::now() - startTime).toSec();

    for (int i = 0; i < numberOfJoints; ++i)
    {
        double velocity = calculateVelocity(actualJointAngles[i],
                                            actualJointVelocities[i],
                                            trajectoryComposite[i],
                                            elapsedTime);

        velocities.push_back(velocity);

    }

}

void JointTrajectoryAction::setTargetTrajectory(double angle1,
                                                double angle2,
                                                double duration,
                                                KDL::Trajectory_Composite& trajectoryComposite)
{

    KDL::Frame pose1(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(angle1, 0, 0));
    KDL::Frame pose2(KDL::Rotation::RPY(0, 0, 0), KDL::Vector(angle2, 0, 0));

    KDL::Path_Line* path = new KDL::Path_Line(pose1, pose2, new KDL::RotationalInterpolation_SingleAxis(), 0.001);
    KDL::VelocityProfile_Spline* velprof = new KDL::VelocityProfile_Spline();

    velprof->SetProfileDuration(0, path->PathLength(),
                                duration);

    KDL::Trajectory_Segment* trajectorySegment = new KDL::Trajectory_Segment(path, velprof);
    trajectoryComposite.Add(trajectorySegment);
}

void JointTrajectoryAction::execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as)
{

    current_state.name = goal->trajectory.joint_names;
    current_state.position.resize(current_state.name.size());
    current_state.velocity.resize(current_state.name.size());
    current_state.effort.resize(current_state.name.size());

    sensor_msgs::JointState angle1;
    angle1.name = goal->trajectory.joint_names;
    angle1.position.resize(angle1.name.size());
    angle1.velocity.resize(angle1.name.size());
    angle1.effort.resize(angle1.name.size());

    sensor_msgs::JointState angle2;
    angle2.name = goal->trajectory.joint_names;
    angle2.position.resize(angle2.name.size());
    angle2.velocity.resize(angle2.name.size());
    angle2.effort.resize(angle2.name.size());

    const uint numberOfJoints = current_state.name.size();
    KDL::Trajectory_Composite trajectoryComposite[numberOfJoints];

    angle2.position = goal->trajectory.points.at(0).positions;
    for (uint i = 1; i < goal->trajectory.points.size(); i++)
    {
        angle1.position = angle2.position;
        angle2.position = goal->trajectory.points.at(i).positions;

        double duration = (goal->trajectory.points.at(i).time_from_start -
                goal->trajectory.points.at(i - 1).time_from_start).toSec();

        for (uint j = 0; j < numberOfJoints; ++j)
        {
            setTargetTrajectory(angle1.position.at(j),
                                angle2.position.at(j),
                                duration,
                                trajectoryComposite[j]);

        }
    }

    const double dt = 1.0 / getFrequency();
    std::vector<double> velocities;
    ros::Time startTime = ros::Time::now();

    for (double time = 0; time <= trajectoryComposite[0].Duration(); time = time + dt)
    {

        controlLoop(current_state.position,
                    current_state.velocity,
                    trajectoryComposite,
                    numberOfJoints,
                    startTime,
                    velocities);

        brics_actuator::JointVelocities command;
        std::vector <brics_actuator::JointValue> armJointVelocities;
        armJointVelocities.resize(current_state.name.size());

        for (uint j = 0; j < numberOfJoints; j++)
        {
            armJointVelocities[j].joint_uri = current_state.name.at(j);
            armJointVelocities[j].value = velocities[j];
            armJointVelocities[j].unit = boost::units::to_string(boost::units::si::radian_per_second);
        }

        command.velocities = armJointVelocities;
        jointStateObserver->updateVelocity(command);

        ros::Duration(dt).sleep();
    }

    sensor_msgs::JointState goal_state;
    goal_state.name = goal->trajectory.joint_names;
    goal_state.position = goal->trajectory.points.back().positions;

    brics_actuator::JointPositions command;
    std::vector <brics_actuator::JointValue> armJointPositions;
    armJointPositions.resize(current_state.name.size());

    for (uint j = 0; j < numberOfJoints; j++)
    {
        armJointPositions[j].joint_uri = current_state.name.at(j);
        armJointPositions[j].value = goal_state.position.at(j);
        armJointPositions[j].unit = boost::units::to_string(boost::units::si::radian);
    }

    command.positions = armJointPositions;
    jointStateObserver->updatePosition(command);
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    as->setSucceeded(result);
}

void JointTrajectoryAction::jointStateCallback(const sensor_msgs::JointState& joint_state)
{
    int k = current_state.name.size();

    if (k != 0)
    {
        for (uint i = 0; i < joint_state.name.size(); i++)
        {
            for (uint j = 0; j < current_state.name.size(); j++)
            {
                if (current_state.name.at(j) == joint_state.name.at(i))
                {
                    current_state.position[j] = joint_state.position.at(i);
		    current_state.velocity[j] = joint_state.velocity.at(i);
                    k--;
                    break;
                }
            }
            if (k == 0)
            {
                break;
            }
        }
    }
}


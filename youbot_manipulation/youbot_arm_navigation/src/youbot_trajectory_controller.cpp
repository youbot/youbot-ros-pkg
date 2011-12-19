/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Dominik Lechner
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

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <control_msgs/FollowJointTrajectoryActionGoal.h>

using namespace std;

static const double INIT_POS[] = {2.94961, 1.352, -2.591, 0.1, 0.1};
static const string JOINTNAME_PRE = "arm_joint_";
static const uint NUM_ARM_JOINTS = 5;
ros::Publisher armPositionsPublisher;
vector<control_msgs::FollowJointTrajectoryActionGoal::ConstPtr> trajectories;

void trajectoryCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
  ROS_INFO("callback: Trajectory received");
  //cout << "Msg-Header" << endl << msg->header << endl;
  trajectories.push_back(msg);
}

void moveToInitPos(brics_actuator::JointPositions &command)
{
  std::stringstream jointName;

  for (int i = 0; i < NUM_ARM_JOINTS; ++i)
  {
    jointName.str("");
    jointName << JOINTNAME_PRE << (i + 1);

    command.positions[i].joint_uri = jointName.str();
    command.positions[i].value = INIT_POS[i];

    command.positions[i].unit = boost::units::to_string(boost::units::si::radians);
    cout << "Joint " << command.positions[i].joint_uri << " = " << command.positions[i].value << " " << command.positions[i].unit << endl;
  };

  cout << "sending command for init position... and wait" << endl;

  //command.positions = armJointPositions;

  cout << command << endl;
  armPositionsPublisher.publish(command);
  ROS_INFO("End of Init-Pos");
  //ros::Duration(5).sleep();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "youbot_trajectory_controller");
	ros::NodeHandle n;
  uint loop_counter = 0;
  brics_actuator::JointPositions command;
  vector <brics_actuator::JointValue> armJointPositions;
  armJointPositions.resize(NUM_ARM_JOINTS);
  command.positions = armJointPositions;

  armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
  ros::spinOnce();

  ros::Subscriber armTrajectory;
  armTrajectory = n.subscribe("/arm_controller/follow_joint_trajectory/goal", 1, trajectoryCallback);

  ros::Duration(1).sleep();
  ros::spinOnce();

  moveToInitPos(command);
  //ros::Duration(10).sleep();
  ROS_INFO("Init Pos should be reached");


  while (n.ok())
  {
    if(!trajectories.empty())
    {
      control_msgs::FollowJointTrajectoryActionGoal::ConstPtr act_msg;
      ROS_INFO("new Trajectory");
      act_msg = trajectories.front();
      trajectories.erase(trajectories.begin());
      cout << "Msg-Header" << endl << *act_msg << endl;

      armJointPositions.resize(act_msg->goal.trajectory.joint_names.size());
      for(int i = 0; i<act_msg->goal.trajectory.joint_names.size();i++)
      {
        armJointPositions[i].joint_uri = act_msg->goal.trajectory.joint_names[i];
        armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
      }
      command.positions = armJointPositions;

      uint pos_count = 0;

      /// prepare first point
      for(int i = 0; i<act_msg->goal.trajectory.joint_names.size();i++)
      {
        command.positions[i].value = act_msg->goal.trajectory.points[pos_count].positions[i];
      }

      ros::Time start_time = ros::Time::now();

      do
      {
        // send point
        armPositionsPublisher.publish(command);

        // prepare next point
        pos_count++;
        for(int i = 0; i<act_msg->goal.trajectory.joint_names.size();i++)
        {
          command.positions[i].value = act_msg->goal.trajectory.points[pos_count].positions[i];
        }

        //sleep
        ros::Duration((start_time+act_msg->goal.trajectory.points[pos_count].time_from_start)-ros::Time::now()).sleep();

      }
      while(pos_count < act_msg->goal.trajectory.points.size()-1);
      armPositionsPublisher.publish(command);
      ros::Duration(0.5).sleep();

    }
    /*
    brics_actuator::JointPositions command;
		vector <brics_actuator::JointValue> armJointPositions;

		armJointPositions.resize(numberOfArmJoints); //TODO:change that
		gripperJointPositions.resize(numberOfGripperJoints);

		std::stringstream jointName;


		// ::io::base_unit_info <boost::units::si::angular_velocity>).name();
		for (int i = 0; i < numberOfArmJoints; ++i) {
			cout << "Please type in value for joint " << i + 1 << endl;
			cin >> readValue;

			jointName.str("");
			jointName << "arm_joint_" << (i + 1);

			armJointPositions[i].joint_uri = jointName.str();
			armJointPositions[i].value = readValue;

			armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;

		};

		cout << "sending command ..." << endl;

		command.positions = armJointPositions;
		armPositionsPublisher.publish(command);

		cout << "--------------------" << endl;
		rate.sleep();
    */
    ros::Duration(1).sleep();
    ros::spinOnce();
    //ROS_INFO("Loop count %d", loop_counter);
    //armPositionsPublisher.publish(command);
    loop_counter++;
	}

	return 0;
}

/* EOF */

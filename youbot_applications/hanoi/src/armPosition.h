/******************************************************************************
* Copyright (c) 2011, Locomotec
*
* Author: Malte Vesper, Steffen Waeldele
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and Modified BSD license. The dual-license implies that
* users of this code may choose which terms they prefer.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for
* more details.
*
******************************************************************************/

#ifndef ARM_ANGLES
#define ARM_ANGLES

#include <ros/ros.h>

#include <vector>
#include <map>
#include <string>

#include "sensor_msgs/JointState.h"
#include "brics_actuator/JointPositions.h"
#include "arm_navigation_msgs/MoveArmActionGoal.h"

namespace hanoi {

class armPosition {
    bool valid;
    std::vector<double> positions;
    std::map<std::string,double> position_mapper;

    ros::NodeHandle node;

    ros::Subscriber subscription;

    ros::Publisher armPublisher;
    ros::Publisher gripperPublisher;
    ros::Publisher armPlanRequestPublisher;

    brics_actuator::JointPositions armPos;
    brics_actuator::JointPositions gripperPos;

    arm_navigation_msgs::MoveArmActionGoal armGoal;

    sensor_msgs::JointState jointState;

    unsigned long idCounter;

    void prepareArmActionGoal();
public:
    armPosition();

    void update(const sensor_msgs::JointState& msg);

    std::vector<double> get(); /** return a vector of sorted anglevlaues, as found in the /arm_1/JointStates msg */
    void setArm(const std::vector<double>& positions); /** sends a position frame for the arm */
    void setGripper(double opening);

    void moveArm(const std::vector<double>& positions); /** runs a trajectory planing and follow that  */
};

}

#endif

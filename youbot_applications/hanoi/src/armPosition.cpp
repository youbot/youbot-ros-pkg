/******************************************************************************
* Copyright (c) 2011, Locomotec
*
* Author: Malte Vesper
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

#include "armPosition.h"

//#include <boost/function.hpp>

#include <iostream>

#include <boost/units/io.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/length.hpp>

#include "arm_navigation_msgs/JointConstraint.h"
#include "geometry_msgs/Pose.h"

#include <assert.h>

namespace hanoi {

void armPosition::prepareArmActionGoal() {
    armGoal.goal.motion_plan_request.group_name = "arm";
    armGoal.goal.motion_plan_request.num_planning_attempts = 1;
    armGoal.goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
    armGoal.goal.motion_plan_request.planner_id = "";

    std::stringstream name;

    for(int i=0;i<5;i++) {
        arm_navigation_msgs::JointConstraint constraint;
        name.str("");
        name << "arm_joint_" << (i+1);
        constraint.joint_name = name.str();
        constraint.tolerance_above = 0.1;
        constraint.tolerance_below = 0.1;
        armGoal.goal.motion_plan_request.goal_constraints.joint_constraints.push_back(constraint);
    }

    armGoal.header.seq = 0; //todo:sinnvoll?
    armGoal.header.stamp = ros::Time::now();
    armGoal.header.frame_id = "/base_link";

    // GoalID
    armGoal.goal_id.stamp = armGoal.header.stamp;
    armGoal.goal.planner_service_name = "ompl_planning/plan_kinematic_path";

    // goal, planning_scene_diff, robot_stategeometry_msgs

    sensor_msgs::JointState& joint = armGoal.goal.planning_scene_diff.robot_state.joint_state;
    joint.header = armGoal.header;


    joint.position.insert(joint.position.end(),8,0);
    joint.velocity.insert(joint.velocity.end(),8,0);
    joint.effort.insert(joint.effort.end(),8,0);

    joint.name.push_back("caster_joint_fl");
    joint.name.push_back("caster_joint_fr");
    joint.name.push_back("caster_joint_bl");
    joint.name.push_back("caster_joint_br");
    joint.name.push_back("wheel_joint_fl");
    joint.name.push_back("wheel_joint_fr");
    joint.name.push_back("wheel_joint_bl");
    joint.name.push_back("wheel_joint_br");

    armGoal.goal.planning_scene_diff.robot_state.multi_dof_joint_state.stamp = armGoal.header.stamp;
    armGoal.goal.planning_scene_diff.robot_state.multi_dof_joint_state.joint_names.push_back("world_joint");
    armGoal.goal.planning_scene_diff.robot_state.multi_dof_joint_state.frame_ids.push_back("base_footprint");
    armGoal.goal.planning_scene_diff.robot_state.multi_dof_joint_state.child_frame_ids.push_back("base_footprint");

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    armGoal.goal.planning_scene_diff.robot_state.multi_dof_joint_state.poses.push_back(pose);


/*


def trajectory_callback(data):
    rospy.loginfo(rospy.get_name()+"I received trajectory armGoal")
    print "Trajectory-armGoal"
    print data
    /*

    #rospy.Subscriber("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, trajectory_callback)

*/
}


armPosition::armPosition() : valid(false), positions(7), idCounter(0) {
    subscription = node.subscribe<const sensor_msgs::JointState&,hanoi::armPosition>("/arm_1/joint_states",1, &armPosition::update,this );
    armPublisher = node.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command",1);
    gripperPublisher = node.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command",1);
    armPlanRequestPublisher = node.advertise<arm_navigation_msgs::MoveArmActionGoal>("/move_arm/goal",1);

    std::stringstream name;

    armPos.positions.resize(5);

    for(int i=0; i<5; i++) {
        name.str("");
        name << "arm_joint_" << (i+1);
        armPos.positions[i].joint_uri = name.str();
        armPos.positions[i].unit = boost::units::to_string(boost::units::si::radians);
    }

    gripperPos.positions.resize(2);
    gripperPos.positions[0].joint_uri = "gripper_finger_joint_l";
    gripperPos.positions[0].unit = boost::units::to_string(boost::units::si::meter);

    gripperPos.positions[1].joint_uri = "gripper_finger_joint_r";
    gripperPos.positions[1].unit = boost::units::to_string(boost::units::si::meter);

    prepareArmActionGoal();

    ros::Duration(1.0).sleep(); //rosmagic to have it run moves

    while(!valid) {
        ros::spinOnce();
    }
}

void armPosition::update(const sensor_msgs::JointState& msg) {
    std::vector<double>::const_iterator it_v = msg.position.begin();
    std::vector<std::string>::const_iterator it_n = msg.name.begin();
    while(it_v < msg.position.end()) {
        position_mapper.insert( std::pair<std::string,double>(*(it_n++), *(it_v++)));
    }

    positions.clear();
    for( std::map<std::string,double>::iterator it = position_mapper.begin(); it != position_mapper.end(); it++ ) {
        positions.push_back(it->second);
    }
    jointState = msg;
    valid=true;
}

std::vector<double> armPosition::get() {
    ros::spinOnce();
    return positions;
}

void armPosition::setArm(const std::vector<double>& positions) {
    assert(positions.size()==5);
    for(int i=0; i<5;i++) {
        armPos.positions[i].value = positions[i];
    }
    armPublisher.publish(armPos);
}

void armPosition::setGripper(double opening) {
    gripperPos.positions[0].value = opening/2.0;
    gripperPos.positions[1].value = opening/2.0;
    gripperPublisher.publish(gripperPos);
}

void armPosition::moveArm(const std::vector<double>& positions) {
    arm_navigation_msgs::MoveArmActionGoal goal = armGoal;

    for(int i=0;i<5;i++) {
        goal.goal.motion_plan_request.goal_constraints.joint_constraints[i].position=positions[i];
    }

    get(); //update position

    sensor_msgs::JointState& joint = goal.goal.planning_scene_diff.robot_state.joint_state;

    for(int i=0;i<jointState.position.size();i++) {
        joint.name.push_back(jointState.name[i]);
        joint.position.push_back(jointState.position[i]);
        joint.velocity.push_back(jointState.velocity[i]);
    }

    joint.effort.insert(joint.effort.end(),jointState.position.size(),0);

    goal.goal_id.id = idCounter++;

    armPlanRequestPublisher.publish(goal);
}

};


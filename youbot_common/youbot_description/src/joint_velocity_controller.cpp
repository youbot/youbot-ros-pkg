/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "robot_mechanism_controllers/joint_velocity_controller.h"
#include "pluginlib/class_list_macros.h"

#include <string>
#include <iostream>


PLUGINLIB_DECLARE_CLASS(youbot_description, JointVelocityController, controller::JointVelocityController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

JointVelocityController::JointVelocityController()
: joint_state_(NULL), command_(0), robot_(NULL), last_time_(0), loop_count_(0)
{
}

JointVelocityController::~JointVelocityController()
{
  sub_command_.shutdown();
}

bool JointVelocityController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
				   const control_toolbox::Pid &pid)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->getTime();

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("JointVelocityController could not find joint named \"%s\"\n",
              joint_name.c_str());
    return false;
  }

  pid_controller_ = pid;

  return true;
}

bool JointVelocityController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;
  robot_ = robot;
  ROS_INFO("Velocity controller started!\n");
  std::string joint_name;
  /*if (!node_.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  if (!(joint_state_ = robot->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint \"%s\" (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

*/


using namespace XmlRpc;
  node_ = n;
  robot_ = robot;

  // Gets all of the joints
  XmlRpc::XmlRpcValue joint_names;
  if (!node_.getParam("joints", joint_names))
  {
    ROS_ERROR("No joints given. (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Malformed joint specification.  (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  for (int i = 0; i < joint_names.size(); ++i)
  {
    XmlRpcValue &name_value = joint_names[i];
    if (name_value.getType() != XmlRpcValue::TypeString)
    {
      ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
                node_.getNamespace().c_str());
      return false;
    }

    pr2_mechanism_model::JointState *j = robot->getJointState((std::string)name_value);
    if (!j) {
      ROS_ERROR("Joint not found: %s. (namespace: %s)",
                ((std::string)name_value).c_str(), node_.getNamespace().c_str());
      return false;
    }
    joints_.push_back(j);
  }

  // Ensures that all the joints are calibrated.
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    if (!joints_[i]->calibrated_)
    {
      ROS_ERROR("Joint %s was not calibrated (namespace: %s)",
                joints_[i]->joint_->name.c_str(), node_.getNamespace().c_str());
      return false;
    }
  }

  for (unsigned int i = 0; i < joints_.size(); ++i ) {
    cout << joints_[i]->joint_->name.c_str() << endl;
  }

  if (!pid_controller_.init(ros::NodeHandle(node_, "pid")))
    return false;

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "state", 1));




  sub_command_ = node_.subscribe("command", 1, &JointVelocityController::setCommandCB, this);

  cout << "WE ARE HERE!" << endl;

  return true;
}


void JointVelocityController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);

}

void JointVelocityController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

std::string JointVelocityController::getJointName()
{
  return joint_state_->joint_->name;
}

// Set the joint velocity command
void JointVelocityController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current velocity command
void JointVelocityController::getCommand(double  & cmd)
{
  cmd = command_;
}

void JointVelocityController::update()
{
  assert(robot_ != NULL);
  ros::Time time = robot_->getTime();

  double error = joint_state_->velocity_ - command_;

  dt_ = time - last_time_;
  double command = pid_controller_.updatePid(error, dt_);
  joint_state_->commanded_effort_ += command;

  cout << "error=" << error << "joint_effort =" << joint_state_->commanded_effort_  <<endl;

  if(loop_count_ % 10 == 0)
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
         controller_state_publisher_->msg_.header.stamp = time;
        for (size_t j = 0; j < joints_.size(); ++j)
      {

      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = joint_state_->velocity_;
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = dt_.toSec();
      controller_state_publisher_->msg_.command = command;
      }

      double dummy;
      getGains(controller_state_publisher_->msg_.p,
               controller_state_publisher_->msg_.i,
               controller_state_publisher_->msg_.d,
               controller_state_publisher_->msg_.i_clamp,
               dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;

  last_time_ = time;
}

void JointVelocityController::setCommandCB(/*const std_msgs::Float64ConstPtr& msg*/const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    trajectory_msgs::JointTrajectoryPoint point;
    point = msg->points[0];
    ROS_INFO("%d joints found!\n", msg->joint_names.size());
    for (int i = 0; i < static_cast<int>(msg->joint_names.size()); ++i) {
	//	ROS_INFO("joint = %s", msg->joint_names[i]);
        string name = msg->joint_names[i];
        double velocity = point.velocities[i];
        double position = point.positions[i];
        cout << "Joint = " << name << " velocity = " <<  velocity << " positions = " << position << endl;
	}
  command_ = point.velocities[0];

}

} // namespace

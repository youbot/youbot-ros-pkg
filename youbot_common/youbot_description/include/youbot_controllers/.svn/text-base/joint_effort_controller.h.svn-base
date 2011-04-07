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

#ifndef JOINT_EFFORT_CONTROLLER_H
#define JOINT_EFFORT_CONTROLLER_H

/**
   @class pr2_controller_interface::JointEffortController
   @brief Joint Effort Controller (torque or force)

   This class passes the commanded effort down through the
   transmissions and safety code.

   @section ROS ROS interface

   @param type Must be "JointEffortController"
   @param joint Name of the joint to control.

   Subscribes to:
   - @b command (std_msgs::Float64) : The joint effort to apply
*/

#include <boost/thread/condition.hpp>
#include <ros/node_handle.h>
#include <pr2_controller_interface/controller.h>

#include <std_msgs/Float64.h>


namespace controller
{

class JointEffortController : public pr2_controller_interface::Controller
{
public:

  JointEffortController();
  ~JointEffortController();

  bool init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name);
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  virtual void starting() { command_ = 0.0;}
  virtual void update();

  pr2_mechanism_model::JointState *joint_state_;

  double command_;

private:
  pr2_mechanism_model::RobotState *robot_;

  ros::NodeHandle node_;
  ros::Subscriber sub_command_;
  void commandCB(const std_msgs::Float64ConstPtr& msg);

};

}

#endif

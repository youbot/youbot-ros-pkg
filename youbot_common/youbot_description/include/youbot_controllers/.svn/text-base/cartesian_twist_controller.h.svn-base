/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
   @class pr2_controller_interface::CartesianTwistController
   @author Wim Meeussen

   @brief Cartesian twist controller

   Controls the twist at the end effector of a chain of the robot.

   @section ROS ROS interface

   @param type Must be "CartesianTwistController"

   @param root_name The name of the root link of the chain of links
   that you wish to control.

   @param tip_name The name of the tip link (end effector) of the
   chain of links that you wish to control.

   @param fb_trans The gains for the PID loop around linear velocity.  See: control_toolbox::Pid

   @param fb_rot The gains for the PID loop around angular velocity.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (geometry_msgs::Twist) : The desired twist to
     achieve.
 */

#ifndef CARTESIAN_TWIST_CONTROLLER_H
#define CARTESIAN_TWIST_CONTROLLER_H

#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>

#include <control_toolbox/pid.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_datatypes.h>


namespace controller {

class CartesianTwistController : public pr2_controller_interface::Controller
{
public:
  CartesianTwistController();
  ~CartesianTwistController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  void starting();
  void update();

  // input of the controller
  KDL::Twist twist_desi_, twist_meas_;

private:
  ros::NodeHandle node_;
  ros::Subscriber sub_command_;
  void command(const geometry_msgs::TwistConstPtr& twist_msg);
  double ff_trans_, ff_rot_;
  ros::Time last_time_;

  // pid controllers
  std::vector<control_toolbox::Pid> fb_pid_controller_;

  // robot description
  pr2_mechanism_model::RobotState *robot_state_;
  pr2_mechanism_model::Chain chain_;

  // kdl stuff for kinematics
  KDL::Chain             kdl_chain_;
  boost::scoped_ptr<KDL::ChainFkSolverVel> jnt_to_twist_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArrayVel       jnt_posvel_;
  KDL::JntArray       jnt_eff_;
  KDL::Jacobian jacobian_;
  KDL::Wrench wrench_out_;

  geometry_msgs::Twist twist_msg_;
};

} // namespace


#endif

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
   @class pr2_controller_interface::CartesianPoseController
   @author Wim Meeussen

   @brief Cartesian pose controller

   Controls the pose of the end effector of a chain of the robot.

   @section ROS ROS interface

   @param type Must be "CartesianPoseController"

   @param root_name The name of the root link of the chain of links
   that you wish to control.

   @param tip_name The name of the tip link (end effector) of the
   chain of links that you wish to control.

   @param output The name of the CartesianWrenchController which will
   achieve the desired wrench computed by this controller.

   @param fb_trans The gains for the PID loop around position.  See: control_toolbox::Pid

   @param fb_rot The gains for the PID loop around orientation.  See: control_toolbox::Pid

   Subscribes to:

   - @b command (geometry_msgs::PoseStamped) : The desired pose to
     achieve.  The controller will transform this into the root link
     (using tf), and then attempt to move the tip link to the
     transformed pose.

   Publishes:

   - @b state/error (geometry_msgs::Twist) : The error from the measured pose to the desired pose.

   - @b state/pose (geometry_msgs::PoseStamped) : The current measured pose of the tip link.
 */

#ifndef CARTESIAN_POSE_CONTROLLER_H
#define CARTESIAN_POSE_CONTROLLER_H

#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include <ros/node_handle.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <control_toolbox/pid.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <message_filters/subscriber.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


namespace controller {

class CartesianPoseController : public pr2_controller_interface::Controller
{
public:
  CartesianPoseController();
  ~CartesianPoseController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  void starting();
  void update();

  void command(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

  // input of the controller
  KDL::Frame pose_desi_, pose_meas_;
  KDL::Twist twist_ff_;

  // state output
  KDL::Twist twist_error_;

private:
  KDL::Frame getPose();

  ros::NodeHandle node_;
  std::string controller_name_, root_name_;
  ros::Time last_time_;

  // robot structure
  pr2_mechanism_model::RobotState *robot_state_;
  pr2_mechanism_model::Chain chain_;

  // pid controllers
  std::vector<control_toolbox::Pid> pid_controller_;

  // kdl stuff for kinematics
  KDL::Chain             kdl_chain_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArray          jnt_pos_;
  KDL::JntArray       jnt_eff_;
  KDL::Jacobian jacobian_;

  // reatltime publisher
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> > state_error_publisher_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > state_pose_publisher_;
  unsigned int loop_count_;

  tf::TransformListener tf_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub_command_;
  boost::scoped_ptr<tf::MessageFilter<geometry_msgs::PoseStamped> > command_filter_;
  //boost::scoped_ptr<tf::MessageNotifier<geometry_msgs::PoseStamped> > command_notifier_;
};

} // namespace


#endif

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

/*!
  \author Stuart Glaser

  \class pr2_controller_interface::JointSplineTrajectoryController

*/

#ifndef JOINT_TRAJECTORY_CONTROLLER_H__
#define JOINT_TRAJECTORY_CONTROLLER_H__

#include <vector>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <ros/node_handle.h>
#include <control_toolbox/pid.h>
#include <pr2_controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>

#include "trajectory_msgs/JointTrajectory.h"
//#include "robot_mechanism_controllers/Trajectory.h"
#include "pr2_controllers_msgs/QueryTrajectoryState.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"

namespace controller {

class JointSplineTrajectoryController : public pr2_controller_interface::Controller
{
public:

  JointSplineTrajectoryController();
  ~JointSplineTrajectoryController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  void starting();
  void update();

private:
  int loop_count_;
  pr2_mechanism_model::RobotState *robot_;
  ros::Time last_time_;
  std::vector<pr2_mechanism_model::JointState*> joints_;
  std::vector<control_toolbox::Pid> pids_;

  ros::NodeHandle node_;

  void commandCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);
  ros::Subscriber sub_command_;

  bool queryStateService(pr2_controllers_msgs::QueryTrajectoryState::Request &req,
                         pr2_controllers_msgs::QueryTrajectoryState::Response &resp);
  ros::ServiceServer serve_query_state_;

  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
      pr2_controllers_msgs::JointTrajectoryControllerState> > controller_state_publisher_;


  // ------ Mechanisms for passing the trajectory into realtime

  // coef[0] + coef[1]*t + ... + coef[5]*t^5
  struct Spline
  {
    std::vector<double> coef;

    Spline() : coef(6, 0.0) {}
  };

  struct Segment
  {
    double start_time;
    double duration;
    std::vector<Spline> splines;
  };
  typedef std::vector<Segment> SpecifiedTrajectory;

  realtime_tools::RealtimeBox<
    boost::shared_ptr<const SpecifiedTrajectory> > current_trajectory_box_;

  // Holds the trajectory that we are currently following.  The mutex
  // guarding current_trajectory_ is locked from within realtime, so
  // it may only be locked for a bounded duration.
  //boost::shared_ptr<const SpecifiedTrajectory> current_trajectory_;
  //boost::recursive_mutex current_trajectory_lock_RT_;

  std::vector<double> q, qd, qdd;  // Preallocated in init

  // Samples, but handling time bounds.  When the time is past the end
  // of the spline duration, the position is the last valid position,
  // and the derivatives are all 0.
  static void sampleSplineWithTimeBounds(const std::vector<double>& coefficients, double duration, double time,
                                         double& position, double& velocity, double& acceleration);
};

}

#endif

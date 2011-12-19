#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_arm_navigation')
import rospy
import string

from std_msgs.msg import String
from arm_navigation_msgs.msg import MoveArmActionGoal, JointConstraint
from control_msgs.msg import FollowJointTrajectoryActionGoal
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from threading import Lock

seq = 0
goal_pos = [2.87404, 1.786, -1.802, 3.139, 0]
init_pos = [2.94961, 1.352, -2.591, 0, 0]
actual_joint_state = JointState()
jointState_Lock = Lock()

wheel_joints = ["caster_joint_fl", "caster_joint_fr", "caster_joint_bl", "caster_joint_br", "wheel_joint_fl", "wheel_joint_fr", "wheel_joint_bl", "wheel_joint_br"]

def jointState_callback(data):
    global actual_joint_state
    jointState_Lock.acquire()
    actual_joint_state = data
    jointState_Lock.release()


def gen_goal_msg():
    global seq
    msg = MoveArmActionGoal()

    # goal motion_plan_request
    
    msg.goal.motion_plan_request.group_name = "arm"
    msg.goal.motion_plan_request.num_planning_attempts = 1
    msg.goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
    msg.goal.motion_plan_request.planner_id = ""
    
    i = 1
    while i<6:
      constraint = JointConstraint()
      constraint.joint_name = 'arm_joint_{0}'.format(i)
      try:
        line = raw_input('{0}: '.format(constraint.joint_name))                
      except EOFError:                      
        continue
      else: 
        constraint.position = string.atof(line)
        i += 1
      #constraint.position = goal_pos[i-1]
      constraint.tolerance_above = 0.1
      constraint.tolerance_below = 0.1
      msg.goal.motion_plan_request.goal_constraints.joint_constraints.append(constraint)

    
    # Header
    msg.header.seq = seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/base_link"
    
    # GoalID
    msg.goal_id.stamp = msg.header.stamp
    msg.goal_id.id = 'youbot_joint_goal_{0}'.format(rospy.get_time()) #seq) 
    msg.goal.planner_service_name = "ompl_planning/plan_kinematic_path"
    
    # goal, planning_scene_diff, robot_state
    joint_state = JointState()
    joint_state.header = msg.header
    
    """
    for i in range(1,6):
      joint_state.name.append('arm_joint_{0}'.format(i))
      joint_state.position.append(init_pos[i-1])
      joint_state.velocity.append(0)
      joint_state.effort.append(0)
    joint_state.name.append("gripper_finger_joint_l")
    joint_state.position.append(0)
    joint_state.velocity.append(0)
    joint_state.effort.append(0)
    joint_state.name.append("gripper_finger_joint_r")
    joint_state.position.append(0)
    joint_state.velocity.append(0)
    joint_state.effort.append(0)   
    """
    jointState_Lock.acquire()
    for i in range(len(actual_joint_state.name)):
      joint_state.name.append(actual_joint_state.name[i])
      joint_state.position.append(actual_joint_state.position[i])
      joint_state.velocity.append(actual_joint_state.velocity[i])
      joint_state.effort.append(0)
    jointState_Lock.release()
    
    for i in range(8):
      joint_state.name.append(wheel_joints[i])
      joint_state.position.append(0)
      joint_state.velocity.append(0)
      joint_state.effort.append(0)
    
    msg.goal.planning_scene_diff.robot_state.joint_state = joint_state  
    
    msg.goal.planning_scene_diff.robot_state.multi_dof_joint_state.stamp = msg.header.stamp
    msg.goal.planning_scene_diff.robot_state.multi_dof_joint_state.joint_names.append("world_joint")
    msg.goal.planning_scene_diff.robot_state.multi_dof_joint_state.frame_ids.append("base_footprint")
    msg.goal.planning_scene_diff.robot_state.multi_dof_joint_state.child_frame_ids.append("base_footprint")
    pose =  Pose()
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    msg.goal.planning_scene_diff.robot_state.multi_dof_joint_state.poses.append(pose)          
    
    seq += 1
    return msg

def trajectory_callback(data):
    rospy.loginfo(rospy.get_name()+"I received trajectory msg")
    print "Trajectory-Msg"
    print data


def talker():
    rospy.init_node('trajectory_planning')
    
    pub = rospy.Publisher('/move_arm/goal', MoveArmActionGoal)
    #rospy.Subscriber("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, trajectory_callback)
    rospy.Subscriber("/arm_1/joint_states", JointState, jointState_callback)
    rospy.sleep(1)
    msg_to_send = gen_goal_msg()
    print "Goal-Msg"
    print msg_to_send
    
    pub.publish(msg_to_send)
    
    #rospy.spin()
    
    #"""
    while not rospy.is_shutdown():
        #str = "hello world %s"%rospy.get_time()
        #print str
        #rospy.loginfo(str)
        #pub.publish(String(str))
        raw_input("Press Enter to continue...")
        #msg_to_send.goal_id.id = 'youbot_joint_goal_{0}'.format(rospy.get_time())
        msg_to_send = gen_goal_msg()
        pub.publish(msg_to_send)
        #rospy.spinOnce()
        rospy.sleep(1)
    #"""

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass


#! /usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Scott Niekum
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Scott Niekum

import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy 
import actionlib as al 
from pr2_controllers_msgs.msg import * 
from trajectory_msgs.msg import *
import arm_navigation_msgs.srv
import kinematics_msgs.srv 
from cartesianTrajIK import *
import copy
import threading
import geometry_msgs
#from pr2_ar_head_track_action.msg import *
import sys
from std_msgs.msg import *
import numpy as np
import numpy.linalg as la
import singleton


class ArmMoveUtils:
    #0=right, 1=left
    def __init__(self, whicharm):
        #Set up right/left arm variables
        self.whicharm = whicharm
        if(whicharm == 0):
            traj_serv_name = '/r_arm_controller/joint_trajectory_action'
            traj_segment_name = 'r_arm_controller/current_segment'
            gripper_topic_name = '/r_gripper_controller/state'
            gripper_serv_name = '/r_gripper_controller/gripper_action'
            pos_topic_name = '/r_arm_controller/state'
            joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
            self.link_name = 'r_wrist_roll_link'
            gripper_traj_serv_name = '/r_gripper_traj_action'
        else:
            traj_serv_name = '/l_arm_controller/joint_trajectory_action'
            traj_segment_name = 'l_arm_controller/current_segment'
            gripper_topic_name = '/l_gripper_controller/state'
            gripper_serv_name = '/l_gripper_controller/gripper_action'
            pos_topic_name = '/l_arm_controller/state'
            joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
            self.link_name = 'l_wrist_roll_link'
            gripper_traj_serv_name = 'l_gripper_traj_action'
        
        self.traj_client = al.SimpleActionClient(traj_serv_name,JointTrajectoryAction)
        while not self.traj_client.wait_for_server(rospy.Duration(5.0)):
            print "Waiting for the joint_trajectory_action server..."
        print "Connected to joint_trajectory_action server"

        rospy.Subscriber(gripper_topic_name, JointControllerState, self.gripperStateCallback)
        self.grip_pos = 0
        self.grip_pos_dot = 0

        self.gripper_client = al.SimpleActionClient(gripper_serv_name, Pr2GripperCommandAction)
        while not self.gripper_client.wait_for_server(rospy.Duration(5.0)):
            print "Waiting for the gripper action server..."
        print "Connected to gripper action server"
        self.grip_goal = Pr2GripperCommandGoal()
        
        #Connect to the gripper trajectory action server
        self.gripper_traj_client = al.SimpleActionClient(gripper_traj_serv_name, Pr2GripperTrajAction)
        while not self.gripper_traj_client.wait_for_server(rospy.Duration(5.0)):
            print "Waiting for the gripper traj action server..."
        print "Connected to gripper traj action server"
        self.grip_traj_goal = Pr2GripperTrajGoal()
        self.grip_traj_goal.gripper_traj = []
         
        self.goal = JointTrajectoryGoal()
        self.goal.trajectory.joint_names = joint_names;
        
        self.pos_lock = threading.Lock()
        self.cart_pos_lock = threading.Lock()
        self.curr_pos = [0.0]*7
        self.curr_cart_pos = [0.0]*7

        rospy.Subscriber(traj_segment_name, Int32, self.trajSegmentCallback)
        self.curr_traj_point = -1

        self.cart_exec = CartesianTrajExecIK(whicharm)
        rospy.Subscriber(pos_topic_name, JointTrajectoryControllerState, self.armPosCallback)


    def gripperStateCallback(self, msg):
        self.grip_pos = msg.process_value
        self.grip_pos_dot = msg.process_value_dot


    def armPosCallback(self, msg):
        self.pos_lock.acquire()
        self.curr_pos = msg.actual.positions
        self.pos_lock.release()
        

    def gripposCallback(self, msg):
        self.curr_grip_sp = msg.set_point 
        self.curr_grip_process_value = msg.process_value
        self.curr_grip_process_value_dot = msg.process_value_dot
        
        
    #Callback telling us what segment point the jointTrajectory controller is on, so that we can sync to it
    def trajSegmentCallback(self, msg):
        self.curr_traj_point = msg.data    
        
    ## @brief Move the joints to specified joint angles
    ## @param angles The set of angles to move to
    def moveToJointAngle(self, angles):
        jp = JointTrajectoryPoint()
        jp.positions = angles
        jp.time_from_start = rospy.Duration(1);
        self.goal.trajectory.points.append(jp)
        self.goal.trajectory.header.stamp = rospy.Time.now()
        self.traj_client.send_goal(self.goal)
        self.traj_client.wait_for_result()
        self.goal.trajectory.points = []
        
    
    #send a command to the gripper action 
    def commandGripper(self, position, max_effort, blocking = 0):
        self.grip_goal.command.position = position
        self.grip_goal.command.max_effort = max_effort
        self.gripper_client.send_goal(self.grip_goal)
        
        #if blocking, wait for the gripper to finish
        if blocking:
            self.gripper_client.wait_for_result()
            
    
    #Cancels a gripper trajectory, typically originally sent by cartesianTrajIK
    def cancelGripperTraj(self):
        self.grip_traj_goal.gripper_traj = []
        self.grip_traj_goal.dt = 0.1
        self.gripper_traj_client.send_goal(self.grip_traj_goal)
            
            
    #Wrapper for commanding gripper actions
    def makeGripperRequest(self, setpt, blocking):
        self.commandGripper(setpt, -1, blocking)

        
    def followJointTraj(self, angles_list, dt):
        for i in range(len(angles_list)):
            jp = JointTrajectoryPoint()
            jp.positions = angles_list[i]
            jp.time_from_start = rospy.Duration(dt*(i+1))
            self.goal.trajectory.points.append(jp)
            
        self.goal.trajectory.header.stamp = rospy.Time.now()
        self.traj_client.send_goal(self.goal)
        self.traj_client.wait_for_result()
        self.goal.trajectory.points = []
        
        
    def followJointTrajPlan(self, plan):
        for i in range(len(plan.points)):
            jp = JointTrajectoryPoint()
            jp.positions = plan.points[i].positions
            jp.velocities = plan.points[i].velocities
            jp.time_from_start = rospy.Duration(plan.times[i])
            self.goal.trajectory.points.append(jp)
        
        print "Sending joint trajectory for execution..."    
        self.goal.trajectory.header.stamp = rospy.Time.now()
        self.traj_client.send_goal(self.goal)
        self.traj_client.wait_for_result()
        self.goal.trajectory.points = []
        print "Joint trajectory done"
    
    
    def followCartTraj(self, x_vec, grip_traj, dt, splice_time, blocking):
        self.pos_lock.acquire()
        start_angles = copy.deepcopy(self.curr_pos)
        self.pos_lock.release()
        self.curr_traj_point = -1
        
        self.cart_exec.followCartTraj(x_vec, grip_traj, dt, start_angles, splice_time, blocking)
    
        
    def followCartTrajPlan(self, plan, grip_traj, dt, splice_time, blocking):
        x_vec = []
        
        for i in range(len(plan.points)):
            x_vec.append(list(plan.points[i].positions[0:7]))
            
        self.pos_lock.acquire()
        start_angles = copy.deepcopy(self.curr_pos)
        self.pos_lock.release()
        self.curr_traj_point = -1

        self.cart_exec.followCartTraj(x_vec, grip_traj, dt, start_angles, splice_time, blocking)
    
    
    def getCartTrajStatsAtTime(self,time):
        delay_ind = self.cart_exec.getOrigTrajPointAtTime(time)
        [delay_joints, delay_grip] = self.cart_exec.getPredictedJointPoseAtTime(time)
        [delay_cart_vel, delay_grip_vel] = self.cart_exec.getPredictedCartVelAtTime(time)
        return [delay_ind, delay_joints, delay_grip, delay_cart_vel, delay_grip_vel]
    
    
    def getCurrTrajPoint(self):
        return self.curr_traj_point
    

    #Returns the gripper pose and pose_dot
    def getGripPoseInfo(self):
        return (self.grip_pos, self.grip_pos_dot)
        
        
        
#Singleton class that handles movement for the arms and head        
class MoveUtils():
    
    __metaclass__ = singleton.Singleton
    
    def __init__(self):
        self.arm = [ArmMoveUtils(i) for i in range(2)]
        
        #self.ar_head_track_client = al.SimpleActionClient('/ar_head_track_action', Pr2ARHeadTrackAction)
        #while not self.ar_head_track_client.wait_for_server(rospy.Duration(5.0)):
        #    print "Waiting for the AR head track action server..."
        #print "Connected to the AR head track action server"
        #self.ar_head_track_goal = Pr2ARHeadTrackGoal()
        
        
    #send a command to the AR head track action 
    #def commandARHeadTrack(self, tag_id):
    #    self.ar_head_track_goal.tag_id = tag_id
    #    self.ar_head_track_client.send_goal(self.ar_head_track_goal)
        
        

if __name__ == '__main__':   
    rospy.init_node('WorldModelNode')    
    mu = MoveUtils()

        
        
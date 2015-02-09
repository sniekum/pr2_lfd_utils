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

import rospy 
import numpy as np
from ar_track_alvar.msg import *
from pr2_controllers_msgs.msg import * 
import geometry_msgs
import threading
from pr2_lfd_utils import singleton
from pr2_lfd_utils import generalUtils
from pr2_lfd_utils import kinematicsUtils
from pr2_lfd_utils.msg import *
import sys


#Singleton class representing object data (from AR tags) and arm states
class ARWorldModel:

    __metaclass__ = singleton.Singleton

    def __init__(self):
        
        self.gen_utils = generalUtils.GeneralUtils()
        self.kinematics_utils = kinematicsUtils.KinematicsUtils()
        
        self.obj_lock = threading.Lock()
        self.objects = dict()
        
        self.pose_locks = threading.Lock()
        self.gripper_locks = threading.Lock()
        self.arm_cart_poses = [[] for i in xrange(2)]
        self.grip_pos = [None]*2
        self.grip_pos_dot = [None]*2

        #Set up right/left pose topic subscriptions
        rospy.Subscriber('/r_arm_controller/state', JointTrajectoryControllerState, self.rArmPosCallback)
        rospy.Subscriber('/l_arm_controller/state', JointTrajectoryControllerState, self.lArmPosCallback)

        rospy.Subscriber('/l_gripper_controller/state', JointControllerState, self.lGripperStateCallback)
        rospy.Subscriber('/r_gripper_controller/state', JointControllerState, self.rGripperStateCallback)
        
        #Set up AR tag topic subscription
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.arPoseMarkerCallback)
        
        #Set up wm publisher
        self.wm_pub = rospy.Publisher('/ar_world_model', pr2_lfd_utils.msg.WMData)
        
        self.rmp_lock = threading.Lock()
        self.rmp_total_weight = 0.0
        self.rmp_pose = geometry_msgs.msg.PoseStamped()
        self.rmp_which_marker = -1
        self.rmp_reset = False



    def rGripperStateCallback(self, msg):
        with self.gripper_locks:
          self.grip_pos[0] = msg.process_value
          self.grip_pos_dot[0] = msg.process_value_dot


    def lGripperStateCallback(self, msg):
        with self.gripper_locks:
          self.grip_pos[1] = msg.process_value
          self.grip_pos_dot[1] = msg.process_value_dot
    
    
    def rArmPosCallback(self, msg):
        with self.pose_locks:
          joint_pos = list(msg.actual.positions)
          cart_pos = self.kinematics_utils.getFK(joint_pos,0)
          self.arm_cart_poses[0] = list(cart_pos)
    
    
    def lArmPosCallback(self, msg):
        with self.pose_locks:
          joint_pos = msg.actual.positions
          cart_pos = self.kinematics_utils.getFK(joint_pos,1)
          self.arm_cart_poses[1] = list(cart_pos)
    
    
    def arPoseMarkerCallback(self, msg):
        with self.obj_lock:
            seen = set()
            for mark in msg.markers:
                p = [0]*7
                p[0] = mark.pose.pose.position.x
                p[1] = mark.pose.pose.position.y
                p[2] = mark.pose.pose.position.z
                p[3] = mark.pose.pose.orientation.x
                p[4] = mark.pose.pose.orientation.y
                p[5] = mark.pose.pose.orientation.z
                p[6] = mark.pose.pose.orientation.w
                
                #Just keep track of most recent sighting of each object for now
                self.objects[mark.id] = p
                seen.add(mark.id)
                
        #Publish the entire world model
        wm_data = pr2_lfd_utils.msg.WMData()
        wm_data.objects = []
        for (obj_id, obj) in self.objects.items():
            temp = pr2_lfd_utils.msg.WMObject()
            temp.id = obj_id
            temp.pose.pose = self.gen_utils.vecToRosPose(obj)
            if obj_id in seen:
                temp.currently_visible = True
            else:
                temp.currently_visible = False
            wm_data.objects.append(temp)
        self.wm_pub.publish(wm_data)
        
    
    
    #Clear the objects and arm state
    def clear(self):
        with self.obj_lock:
            self.objects = dict()
        with self.pose_locks:
            self.arm_cart_poses[0] = []
        with self.pose_locks:
            self.arm_cart_poses[1] = []
    
    
    #Used for adding observations from outside the library not directly from subscriptions
    #Objects is a dict of (id, pose(7-D)) 
    #Arms is a size 2 list of poses (7-D) and is optional
    def addExternalObservations(self, objects, arms=[]):
        with self.obj_lock:
            for (obj_id, obj) in objects.items():
                self.objects[obj_id] = obj
        
        if arms != []:
            for i in xrange(2):
                with self.pose_locks:#[i]:
                    self.arm_poses[i] = arms[i]
    
    
    #Check if all objects exist in world model
    def checkIfAllExist(self, obj_ids):
        with self.obj_lock:
            return all([obj in self.objects for obj in obj_ids])
            
    
    #Look around with the head until all required objects are in the world model
    def searchUntilAllFound(self, obj_ids):
        for obj in obj_ids:
            if not obj in self.objects:
                self.move_utils.commandARHeadTrack(obj)
                while not obj in self.objects:
                    rospy.sleep(1.0)
        self.move_utils.commandARHeadTrack(-1)
        
    
    #Get all objects poses that world model knows about
    def getFullWorldState(self):
        with self.obj_lock:
            return dict(self.objects)
        
        
    #Get only the objects from the world model corresponding to the given list of object ids  
    def getPartialWorldState(self, obj_ids):
        with self.obj_lock:
            return dict((k,self.objects[k]) for k in obj_ids if k in self.objects) 
            
            
    def getObjectById(self, obj_id):
        with self.obj_lock:
            if obj_id in self.objects:
                return self.objects[obj_id]
            else:
                print "getObjectById: ID not found:", obj_id
                return -1
          
          
    def getArmCartState(self, whicharm):
        with self.pose_locks:
            return list(self.arm_cart_poses[whicharm] + [self.grip_pos[whicharm]])
            
            
            
    
    
   ############################## Code for better marker pose estimation ################################### 
    
    #Set which marker to incrementally estimate a position of over time
    def startEstimateRelativeMarkerPoseIncremental(self, m_id):
        self.rmp_reset = True
        self.rmp_which_marker = m_id
    
    
    def getEstimatedRelativeMarkerPose(self):
        return list(self.rmp_pose)
      
    
    ############# TODO: UNTESTED ################
    #Incrementally estimates pose of marker in wrist frame, weighting by confidence (number of contributing tags to averaged observation)
    #Assumes that the rigid transform between the two doesn't change during averaging period
    #Updates using provided observed pose and marker data
    def estimateRelativeMarkerPoseIncremental(self, wrist_pose, marker_pose, weight, should_reset):
        if should_reset:
            self.rmp_total_weight = 0.0
            self.rmp_pose = geometry_msgs.msg.PoseStamped()
            
        #Make a transformation from torso to the wrist position
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = 'torso_lift_link'
        t.child_frame_id = 'obs_wrist'
        t.transform = gu.vecToRosTransform(wrist_pose)
        self.tformer.setTransform(t) 
        
        #Then get the marker coordinates relative to the first wrist position frame
        marker_torso = geometry_msgs.msg.PoseStamped()
        marker_torso.header.frame_id = 'torso_lift_link'
        marker_torso.pose = gu.vecToRosPose(marker_pose)
        marker_wrist = self.tformer.transformPose('obs_wrist',marker_torso)
        
        pos_list = [self.rmp_pose[0:3], marker_wrist[0:3]]
        quat_list = [self.rmp_pose[0:3], marker_wrist[3:7]]
        w_list = [self.rmp_total_weight, weight]
        pos_and_w = zip(pos_list, w_list)
        self.total_weight += weight
        
        avg_quat = self.averageQuaternions(quat_list, valid_conf)
        avg_pos = [sum([p[i]*w for p,w in pos_and_w])/self.total_weight for i in range(3)]
        
        self.rmp_pose.header.frame_id = 'obs_wrist'
        self.rmp_pose.pose = gu.vecToRosPose(avg_pos + avg_quat)
        return self.rmp_pose
            
     
    #Estimate pose of marker in wrist frame, weighting by confidence (number of contributing tags to averaged observation)
    #Assumes that the rigid transform between the two doesn't change during demonstration
    def estimateRelativeMarkerPoseFromTraj(self, arm_control_data, markerfile, control_frame):
        #Load the marker data 
        marker_data = []
        f = open(markerfile, 'r')
        while 1:
            try: marker_data.append(pickle.load(f))  
            except EOFError: break 
        
        #Create a list of marker poses (or missing data) at each timestep for given ID     
        filtered_marker_poses = [(self.filterMarkers(marks_seen,control_frame))[0] for marks_seen in marker_data]
        filtered_marker_conf = [(self.filterMarkers(marks_seen,control_frame))[1] for marks_seen in marker_data]
        
        valid_marker_poses = [fp for fp in filtered_marker_poses if not fp==[]]
        valid_conf = [fc for fc in filtered_marker_conf if not fc==[]]
        valid_arm_poses = [arm_control_data[i] for i in range(len(arm_control_data)) if not filtered_marker_poses[i]==[]]
        
        #Make the poses relative to the wrist, rather than the torso frame
        marker_poses_wrist = []
        for mp,ap in zip(valid_marker_poses, valid_arm_poses):
            
            #Make a transformation from torso to the wrist position
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = 'torso_lift_link'
            t.child_frame_id = 'curr_wrist'
            t.transform = gu.vecToRosTransform(ap)
            self.tformer.setTransform(t) 
            
            #Then get the marker coordinates relative to the first wrist position frame
            marker_torso = geometry_msgs.msg.PoseStamped()
            marker_torso.header.frame_id = 'torso_lift_link'
            marker_torso.pose = gu.vecToRosPose(mp)
            marker_wrist = self.tformer.transformPose('curr_wrist',marker_torso)
            
            marker_poses_wrist.append(gu.rosPoseToVec(marker_wrist.pose))
        
        pos_list = [mp[0:3] for mp in marker_poses_wrist]
        quat_list = [mp[3:7] for mp in marker_poses_wrist]
        pos_and_w = zip(pos_list, valid_conf)
        total_w = sum(valid_conf)
        
        avg_quat = self.averageQuaternions(quat_list, valid_conf)
        avg_pos = [sum([p[i]*w for p,w in pos_and_w])/total_w for i in range(3)]
        
        marker_avg = geometry_msgs.msg.PoseStamped()
        marker_avg.header.frame_id = 'curr_wrist'
        marker_avg.pose = gu.vecToRosPose(avg_pos + avg_quat)
            
        return marker_avg
        
    
    
        
if __name__ == '__main__':   
    rospy.init_node('WorldModelNode')
    task_objects = [0,8]
    
    wm = ARWorldModel()
    #wm.searchUntilAllFound(task_objects)
    rospy.spin()
    
    

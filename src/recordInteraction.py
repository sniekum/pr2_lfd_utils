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
import numpy as np
import pr2_mechanism_msgs.srv
from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import * 
from sensor_msgs.msg import *
import rosbag
import subprocess
import os
import signal
import time


class RecordInteraction():
    
    def __init__(self, base_path, joint_thresh=0.001):
        
        print 'Waiting for switch controller service...'
        rospy.wait_for_service('pr2_controller_manager/switch_controller')
        self.switch_control = rospy.ServiceProxy('pr2_controller_manager/switch_controller', pr2_mechanism_msgs.srv.SwitchController, persistent=True)
        print 'OK'
        
        self.standard_controllers = ['r_arm_controller', 'l_arm_controller']
        self.mannequin_controllers = ['r_arm_controller_loose', 'l_arm_controller_loose']
        self.whicharm_mann = 1
        
        self.switch_req = pr2_mechanism_msgs.srv.SwitchControllerRequest()
        self.switch_req.strictness = pr2_mechanism_msgs.srv.SwitchControllerRequest.BEST_EFFORT
        
        #Only change fixed point if some joint moves at least joint_thresh radians
        self.joint_bounds = [joint_thresh]*10
        self.interaction = False
        
        self.r_pub = rospy.Publisher("/r_arm_controller_loose/command", trajectory_msgs.msg.JointTrajectory)
        rospy.Subscriber("/r_arm_controller_loose/state", pr2_controllers_msgs.msg.JointTrajectoryControllerState, self.rightJointStateCallback)
        self.l_pub = rospy.Publisher("/l_arm_controller_loose/command", trajectory_msgs.msg.JointTrajectory)
        rospy.Subscriber("/l_arm_controller_loose/state", pr2_controllers_msgs.msg.JointTrajectoryControllerState, self.leftJointStateCallback)
        
        self.bag_process = None
        self.recording = False
        self.base_path = base_path
        self.file_path = base_path
        self.seg_num = 1
        #Use to prevent it from thinking there are many rapid presses due to the checking loop
        self.start_hold = False


        rospy.Subscriber("joy", sensor_msgs.msg.Joy, self.joystick_callback);
        
        


    def __del__(self):
        #Make sure to stop recording bag before shutdown
        self.stopRecord()

    
    def joystick_callback(self, msg):
       
        #Check for Start button to start recording
        if(msg.buttons[3] == 1):
            if self.start_hold == False:
                self.start_hold = True
                self.startInteraction()
                self.startRecord()
        else:
            self.start_hold = False
            
        #Check for Select button to stop recording
        if(msg.buttons[0] == 1):
            self.stopRecord()
            self.stopInteraction()
            
        #Check for left circle button to switch to left arm
        if(msg.buttons[15] == 1):
            self.switchToLeftArm()
            
        #Check for right circle button to switch to right arm
        if(msg.buttons[13] == 1):
            self.switchToRightArm()
                
    
    #Start interaction by stopping standard controllers and going into mannequin mode
    #Starts with right arm mannequin, left arm rigid
    def startInteraction(self):
        if not self.interaction:
            self.switch_req.stop_controllers = [self.standard_controllers[0]]
            self.switch_req.start_controllers = [self.mannequin_controllers[0]]
            resp = self.switch_control(self.switch_req)
            self.whicharm_mann = 0
            self.interaction = True
        
        
    def stopInteraction(self):
        if self.interaction:
            self.switch_req.stop_controllers = [self.mannequin_controllers[self.whicharm_mann]]
            self.switch_req.start_controllers = [self.standard_controllers[self.whicharm_mann]]
            resp = self.switch_control(self.switch_req)
            self.interaction = False
            
            
    def switchToRightArm(self):
        if self.interaction:
            if self.whicharm_mann == 1:
                print "right"
                self.switch_req.stop_controllers = [self.mannequin_controllers[1], self.standard_controllers[0]]
                self.switch_req.start_controllers = [self.standard_controllers[1], self.mannequin_controllers[0]]
                resp = self.switch_control(self.switch_req)
                self.switchRecord()
                self.whicharm_mann = 0
    
    
    def switchToLeftArm(self):
        if self.interaction:
            if self.whicharm_mann == 0:
                print "left"
                self.switch_req.stop_controllers = [self.mannequin_controllers[0], self.standard_controllers[1]]
                self.switch_req.start_controllers = [self.standard_controllers[0], self.mannequin_controllers[1]]
                resp = self.switch_control(self.switch_req)
                self.switchRecord()
                self.whicharm_mann = 1
    
    
    def startRecord(self):
        if not self.recording:
            self.seg_num = 1
            #Make a new folder for the demo named by date/time 
            t = time.localtime()
            self.file_path = self.base_path + str(t.tm_year) + "_" + str(t.tm_mon) + "_" + str(t.tm_mday) + "_" + str(t.tm_hour) + "_" + str(t.tm_min) + "_" + str(t.tm_sec) + "/" 
            os.makedirs(self.file_path)
            
            print "Beginning to record."
            self.recording = True
            
            topics = "/ar_world_model /l_arm_controller_loose/state /r_arm_controller_loose/state /l_gripper_controller/state /r_gripper_controller/state"
            filename = self.file_path + "/part" + str(self.seg_num) + ".bag " 
            command = "rosbag record -O " + filename + topics
            self.bag_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
        else:
            #Increment file name without switching arms.
            self.switchRecord()
            
         
    def stopRecord(self):
        if self.recording:
            print "Stopping recording.\n"
            self.recording = False
            
            #Make sure we kill the child process that rosbag spawns in addition to the parent subprocess
            pid = self.bag_process.pid
            os.killpg(pid, signal.SIGINT)
            
        
    def switchRecord(self):
        #Make sure we kill the child process that rosbag spawns in addition to the parent subprocess
        pid = self.bag_process.pid
        os.killpg(pid, signal.SIGINT)
        
        #Open a new file with incremented segment number
        topics = "/ar_world_model /l_arm_controller_loose/state /r_arm_controller_loose/state /l_gripper_controller/state /r_gripper_controller/state"
        self.seg_num += 1
        filename = self.file_path + "/part" + str(self.seg_num) + ".bag " 
        print "Switching to record file", filename
        command = "rosbag record -O " + filename + topics
        self.bag_process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            
                
    def rightJointStateCallback(self, msg):
        if self.interaction:
            max_error = max([abs(x) for x in msg.error.positions])
            exceeded = [abs(x) > y for x,y in zip(msg.error.positions, self.joint_bounds)]

            if any(exceeded):
                # Copy our current state into the commanded state
                cmd = trajectory_msgs.msg.JointTrajectory()
                cmd.header.stamp = msg.header.stamp
                cmd.joint_names = msg.joint_names
                cmd.points.append( trajectory_msgs.msg.JointTrajectoryPoint())
                cmd.points[0].time_from_start = rospy.Duration(.125)
                cmd.points[0].positions = msg.actual.positions
                self.r_pub.publish(cmd)
    
    
    def leftJointStateCallback(self, msg):
        if self.interaction:
            max_error = max([abs(x) for x in msg.error.positions])
            exceeded = [abs(x) > y for x,y in zip(msg.error.positions, self.joint_bounds)]

            if any(exceeded):
                # Copy our current state into the commanded state
                cmd = trajectory_msgs.msg.JointTrajectory()
                cmd.header.stamp = msg.header.stamp
                cmd.joint_names = msg.joint_names
                cmd.points.append( trajectory_msgs.msg.JointTrajectoryPoint())
                cmd.points[0].time_from_start = rospy.Duration(.125)
                cmd.points[0].positions = msg.actual.positions
                self.l_pub.publish(cmd)
        
        
if __name__ == '__main__':
    rospy.init_node('recordInteractionNode')
    rc = RecordInteraction("test/")
    #rc.startInteraction()
    #rc.stopInteraction()
    rospy.spin()
    



        

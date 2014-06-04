#!/usr/bin/env python
#
# Copyright (c) 2010, Bosch LLC
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of Bosch LLC nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Sebastian Haug, Bosch LLC


import rospy

from pr2_controllers_msgs.msg import *
import actionlib

class Gripper:
    def __init__(self, side):
        if side == 'r':
            self.side = 'r'
        elif side == 'l':
            self.side = 'l'
        else:
            rospy.logerr("Abort. Specify 'l' or 'r' for side!")
            exit(1)
        
        self.gripperClient = actionlib.SimpleActionClient("/" + self.side + "_gripper_controller/gripper_action", Pr2GripperCommandAction)
        
        if not self.gripperClient.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Could not connect to /" + self.side + "_gripper_controller/gripper_action action server.")
            exit(1)
        
    def getGripperPos(self):
        msg = rospy.wait_for_message("/" + self.side + "_gripper_controller/state", pr2_controllers_msgs.msg.JointControllerState)
        return msg.process_value
    
    def closeGripper(self, effort = -1, wait = True):
        command = Pr2GripperCommand()
        command.position = 0.0
        command.max_effort = effort

        text_result = self.execGripperCommand(command, wait)
        rospy.loginfo("Closing " + self.side + " gripper " + text_result)
        
    def openGripper(self, effort = -1, wait = True):
        command = Pr2GripperCommand()
        command.position = 0.08
        command.max_effort = effort
        
        text_result = self.execGripperCommand(command, wait)
        rospy.loginfo("Opening " + self.side + " gripper " + text_result)
        
    def gripperToPos(self, pos, effort = -1, wait = True):
        command = Pr2GripperCommand()
        command.position = pos
        command.max_effort = effort
        
        text_result = self.execGripperCommand(command, wait)
        rospy.loginfo("Moving " + self.side + " gripper to position " + text_result)
    
    def relaxGripper(self, wait = True ):
        command = Pr2GripperCommand()
        command.max_effort = 0
        command.position = self.getGripperPos()
        goal = Pr2GripperCommandGoal(command)
        
        text_result = self.execGripperCommand(command, wait)
        rospy.loginfo("Relaxing " + self.side + " gripper " + text_result)
    
    def execGripperCommand(self, command, wait = True):
        goal = Pr2GripperCommandGoal(command)
        self.gripperClient.send_goal(goal)
        if wait:
            self.gripperClient.wait_for_result()
        
        result = self.gripperClient.get_result()
        did = []
        if self.gripperClient.get_state() != actionlib.GoalStatus.SUCCEEDED:
            did.append("failed!")
        else:
            if result.stalled: did.append("stalled!")
            if result.reached_goal: did.append("succeeded!")
        return ' and '.join(did)

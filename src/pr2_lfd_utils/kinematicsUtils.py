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
# author: Karol Hausman

import rospy 
import threading
from pr2_lfd_utils import singleton
import sys

from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, GetPositionIKRequest


class KinematicsUtils:

    __metaclass__ = singleton.Singleton

    def __init__(self):
        
        ik_serv_name = '/compute_ik'
        fk_serv_name = '/compute_fk'
        self.frame_id = "torso_lift_link"
        self.r_link_name = 'r_wrist_roll_link'
        self.l_link_name = 'l_wrist_roll_link'
        self.r_group_name = 'right_arm'
        self.l_group_name = 'left_arm'
        

        #Set up right/left FK service connections
        print 'Waiting for forward kinematics services...'
        rospy.wait_for_service(fk_serv_name)
        self.getPosFK = rospy.ServiceProxy(fk_serv_name, GetPositionFK)
        self.getPosFKPersistent = rospy.ServiceProxy(fk_serv_name, GetPositionFK, persistent = True)
        print "OK"
        
        #Set up right/left FK service requests
        self.FKreq = [GetPositionFKRequest(), GetPositionFKRequest()]
        self.FKreq[0].header.frame_id = self.frame_id
        self.FKreq[0].fk_link_names = [self.r_link_name]
        self.FKreq[0].robot_state.joint_state.name = self.getJointNames(0)
        self.FKreq[1].header.frame_id = self.frame_id
        self.FKreq[1].fk_link_names = [self.l_link_name]
        self.FKreq[1].robot_state.joint_state.name = self.getJointNames(1)



        #Set up right/left IK service connections
        print 'Waiting for inverse kinematics services...'
        rospy.wait_for_service(ik_serv_name)
        self.getPosIK = rospy.ServiceProxy(ik_serv_name, GetPositionIK)
        self.getPosIKPersistent = rospy.ServiceProxy(ik_serv_name, GetPositionIK, persistent=True)
        print 'OK'

        #Set up right/left IK service requests
        self.IKreq = [GetPositionIKRequest(), GetPositionIKRequest()]
        self.IKreq[0].ik_request.robot_state.joint_state.position = [0]*7
        self.IKreq[0].ik_request.robot_state.joint_state.name = self.getJointNames(0)
        self.IKreq[0].ik_request.group_name = self.r_group_name
        self.IKreq[0].ik_request.ik_link_name = self.r_link_name
        self.IKreq[0].ik_request.pose_stamped.header.frame_id = self.frame_id
        self.IKreq[1].ik_request.robot_state.joint_state.position = [0]*7
        self.IKreq[1].ik_request.robot_state.joint_state.name = self.getJointNames(1)
        self.IKreq[1].ik_request.group_name = self.l_group_name
        self.IKreq[1].ik_request.ik_link_name = self.l_link_name
        self.IKreq[1].ik_request.pose_stamped.header.frame_id = self.frame_id
       
        
    
    #Converts joint angles to cartesian pose of end effector fo the indicated arm (0=right, 1=left)
    def getFK(self, angles, whicharm, persistent = False):
        self.FKreq[whicharm].robot_state.joint_state.position = angles
        try:
            if (persistent):
              response = self.getPosFKPersistent(self.FKreq[whicharm])
            else:
              response = self.getPosFK(self.FKreq[whicharm])
        except rospy.ServiceException, e:
            print "FK service failure: %s" % str(e) 
            sys.exit(0)

        #Get the cartesian pose of the wrist_roll_joint    
        cartPos = response.pose_stamped[0].pose
        r = []
        r.append(cartPos.position.x)
        r.append(cartPos.position.y)
        r.append(cartPos.position.z)
        r.append(cartPos.orientation.x)
        r.append(cartPos.orientation.y)
        r.append(cartPos.orientation.z)
        r.append(cartPos.orientation.w)
        
        return r

    #Converts cartesian position of the end effector to joint angles using start_angles as seed points
    def getIK(self, cart_pos, start_angles, whicharm, persistent = True):
        self.IKreq[whicharm].ik_request.pose_stamped.pose.position.x = cart_pos[0]
        self.IKreq[whicharm].ik_request.pose_stamped.pose.position.y = cart_pos[1]
        self.IKreq[whicharm].ik_request.pose_stamped.pose.position.z = cart_pos[2]
        self.IKreq[whicharm].ik_request.pose_stamped.pose.orientation.x = cart_pos[3]
        self.IKreq[whicharm].ik_request.pose_stamped.pose.orientation.y = cart_pos[4]
        self.IKreq[whicharm].ik_request.pose_stamped.pose.orientation.z = cart_pos[5]
        self.IKreq[whicharm].ik_request.pose_stamped.pose.orientation.w = cart_pos[6]
        self.IKreq[whicharm].ik_request.robot_state.joint_state.position = start_angles
        if (persistent):
          response = self.getPosIKPersistent(self.IKreq[whicharm])
        else:
          response = self.getPosIK(self.IKreq[whicharm])
   
        return response



    
    #Returns joint names for the indicated arm(0=right, 1=left)
    def getJointNames(self, whicharm):
        if (whicharm == 0):
          controller = "r_arm_controller"
        else:
          controller = "l_arm_controller"    
        return rospy.get_param(controller + "/joints");

    def getLinkName(self, whicharm):  
        if (whicharm == 0):
          return self.r_link_name
        else:
          return self.l_link_name

    def getGroupName(self, whicharm):  
        if (whicharm == 0):
          return self.r_group_name
        else:
          return self.l_group_name

    def getFrameId(self):  
        return self.frame_id
    
        
if __name__ == '__main__':   
    rospy.init_node('kinematicsUtils')
    
    kU = kinematicsUtils()
    rospy.spin()
    
    

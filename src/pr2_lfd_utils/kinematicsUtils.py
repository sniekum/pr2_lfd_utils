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

from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest




#Singleton class representing object data (from AR tags) and arm states
class KinematicsUtils:

    __metaclass__ = singleton.Singleton

    def __init__(self):
        
        self.pose_locks = threading.Lock()

        #Set up right/left FK service connections
        print 'Waiting for forward kinematics services...'
        rospy.wait_for_service('/compute_fk')
        self.getPosFK = rospy.ServiceProxy("compute_fk", GetPositionFK)
        print "OK"
        
        #Set up right/left FK service requests
        self.FKreq = [GetPositionFKRequest(), GetPositionFKRequest()]
        self.FKreq[0].header.frame_id = "torso_lift_link"
        self.FKreq[0].fk_link_names = ['r_wrist_roll_link']
        self.FKreq[0].robot_state.joint_state.name = self.getJointNames(0)
        self.FKreq[1].header.frame_id = "torso_lift_link"
        self.FKreq[1].fk_link_names = ['l_wrist_roll_link']
        self.FKreq[1].robot_state.joint_state.name = self.getJointNames(1)
       
        
    
    #Converts joint angles to cartesian pose of end effector fo the indicated arm (0=right, 1=left)
    def getFK(self, angles, whicharm):
        self.FKreq[whicharm].robot_state.joint_state.position = angles
        try:
            response = self.getPosFK(self.FKreq[whicharm])
        except rospy.ServiceException, e:
            print "FK service failure: %s" % str(e) 

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
    
    #Returns joint names for the indicated arm(0=right, 1=left)
    def getJointNames(self, whicharm):
        if (whicharm == 0):
          controller = "r_arm_controller"
        else:
          controller = "l_arm_controller"    
        return rospy.get_param(controller + "/joints");
    
        
if __name__ == '__main__':   
    rospy.init_node('kinematicsUtils')
    
    kU = kinematicsUtils()
    rospy.spin()
    
    

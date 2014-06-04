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

#import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy 
import copy
import geometry_msgs
import tf
import singleton
import numpy as np

#Singleton class that contains generally useful utility functions that don't require ROS subscriptions or service connections
class GeneralUtils():
    
    __metaclass__ = singleton.Singleton

    def rosPoseToVec(self, pose):
        vec = [0]*7
        vec[0] = pose.position.x
        vec[1] = pose.position.y
        vec[2] = pose.position.z
        vec[3] = pose.orientation.x
        vec[4] = pose.orientation.y
        vec[5] = pose.orientation.z
        vec[6] = pose.orientation.w
        return vec



    def vecToRosPose(self, vec):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = vec[0]
        pose.position.y = vec[1]
        pose.position.z = vec[2]
        pose.orientation.x = vec[3]
        pose.orientation.y = vec[4]
        pose.orientation.z = vec[5]
        pose.orientation.w = vec[6]
        return pose
        
    
    
    def rosPointToVec(self, pt):
        vec = [0]*3
        vec[0] = pt.x
        vec[1] = pt.y
        vec[2] = pt.z
        return vec
    
    
        
    def vecToRosPoint(self, vec):
        pt = geometry_msgs.msg.Point()
        pt.x = vec[0]
        pt.y = vec[1]
        pt.z = vec[2]
        return pt



    def rosTransformToVec(self, trans):
        vec = [0]*7
        vec[0] = trans.translation.x
        vec[1] = trans.translation.y
        vec[2] = trans.translation.z
        vec[3] = trans.rotation.x
        vec[4] = trans.rotation.x
        vec[5] = trans.rotation.x
        vec[6] = trans.rotation.x
        return vec
        
        

    def vecToRosTransform(self, vec): 
        trans = geometry_msgs.msg.Transform()
        trans.translation.x = vec[0]
        trans.translation.y = vec[1]
        trans.translation.z = vec[2]
        trans.rotation.x = vec[3]
        trans.rotation.y = vec[4]
        trans.rotation.z = vec[5]
        trans.rotation.w = vec[6]
        return trans
        


    def rosPoseToRosTransform(self, pose):
        trans = geometry_msgs.msg.Transform()
        trans.translation.x = pose.position.x
        trans.translation.y = pose.position.y
        trans.translation.z = pose.position.z
        trans.rotation.x = pose.orientation.x
        trans.rotation.y = pose.orientation.y
        trans.rotation.z = pose.orientation.z
        trans.rotation.w = pose.orientation.w
        return trans
        
        

    #Flips the signs on the quaternion of the given point to be closest to that of the baseline      
    def minimizeQuaternionError(self,p,baseline):
        t1 = np.fabs(p[3] - baseline[3])
        t2 = np.fabs(p[4] - baseline[4])
        t3 = np.fabs(p[5] - baseline[5])
        t4 = np.fabs(p[6] - baseline[6])
        tsum = t1+t2+t3+t4
        
        #check distance to next point with a sign flip
        f1 = np.fabs(p[3] + baseline[3])
        f2 = np.fabs(p[4] + baseline[4])
        f3 = np.fabs(p[5] + baseline[5])
        f4 = np.fabs(p[6] + baseline[6])
        fsum = f1+f2+f3+f4
        
        #choose the representation that causes the least disturbance
        if(fsum < tsum):
            p[3] = -1 * p[3]
            p[4] = -1 * p[4]
            p[5] = -1 * p[5]
            p[6] = -1 * p[6]    
        
        
        
    #Convert quaternion to a canonical representation in a single hemisphere to avoid aliasing
    def makeQuaternionCanonical(self, quat):
        if quat[0] < 0:
            quat = [quat[i]*-1 for i in range(4)]
        elif quat[0] == 0:
            if quat[1] < 0:
                quat = [quat[i]*-1 for i in range(4)]
                
        return quat



    #Performs mathematically correct weighted quaternion averaging for a list of quaternions
    #Algorithm taken from "Averaging Quaternions" by Cheng et al.
    def averageQuaternions(self, quat_list, weights):
        M = np.zeros([4,4])
        
        #Sum the weighted outer products of the quaternions
        for q,w in zip(quat_list, weights):
            a = np.array(q)
            M = M + (w * np.outer(a,a))
            
        #Average quaternion is the eigenvector of the summed matrix with the largest eigenvalue    
        [evals,evecs] = la.eig(M)
        evals_list = evals.flatten().tolist()
        max_ind = evals_list.index(max(evals_list))
        return evecs[:,max_ind].tolist()

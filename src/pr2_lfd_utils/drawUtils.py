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
import matplotlib.pyplot as plt
import singleton
import generalUtils 
import visualization_msgs.msg as vm
import tf
import geometry_msgs.msg
import std_msgs.msg
import numpy as np
from mpl_toolkits.mplot3d import *

#Singleton class to support drawing / plotting operations
class DrawUtils:
    
    __metaclass__ = singleton.Singleton
    
    
    def __init__(self):
        self.rviz_pub = rospy.Publisher('dmp_goal_arrow', vm.Marker)
        self.rviz_pub2 = rospy.Publisher('grip_pos_arrow', vm.Marker)
        self.tf_broad = tf.TransformBroadcaster()
        self.gen_utils = generalUtils.GeneralUtils()
        
    
    def plotPlan(self,plan):
        n_pts = len(plan.points)
        dims = len(plan.points[0].positions)
        
        t_vec = plan.times
        x_vec = [0]*n_pts
        
        plt.figure()
        for i in range(len(plan.points[0].positions)):
            for j in range(n_pts):
                x_vec[j] = plan.points[j].positions[i]
            plt.plot(t_vec, x_vec)
            
     
    def plotTraj(self,traj,dt):
        n_pts = len(traj)
        dims = len(traj[0:3])
        
        t_vec = [0]*n_pts
        x_vec = [0]*n_pts

        
        plt.figure()
        for i in range(dims):
            for j in range(n_pts):
                x_vec[j] = traj[j][i]
                t_vec[j] = dt*j
            plt.plot(t_vec, x_vec)



    def plotTraj3D(self,traj):
        n_pts = len(traj)
        dims = len(traj[0:3])
        
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        x_vec = [0]*n_pts
        y_vec = [0]*n_pts
        z_vec = [0]*n_pts

        for j in range(n_pts):
          x_vec[j] = traj[j][0]
          y_vec[j] = traj[j][1]
          z_vec[j] = traj[j][2]

        ax.scatter(x_vec, y_vec, z_vec)
      




    def plotTwoTraj3D(self,traj1, traj2):

        if (len(traj1) != len(traj2)):
          print "Length of trajectory 1 is not equal to trajectory 2"          
          return
        n_pts = len(traj1)
        dims = len(traj1[0:3])
        
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        x_vec1 = [0]*n_pts
        y_vec1 = [0]*n_pts
        z_vec1 = [0]*n_pts

        x_vec2 = [0]*n_pts
        y_vec2 = [0]*n_pts
        z_vec2 = [0]*n_pts


        for j in range(n_pts):
          x_vec1[j] = traj1[j][0]
          y_vec1[j] = traj1[j][1]
          z_vec1[j] = traj1[j][2]

          x_vec2[j] = traj2[j][0]
          y_vec2[j] = traj2[j][1]
          z_vec2[j] = traj2[j][2]


        ax.scatter(x_vec1, y_vec1, z_vec1)
        ax.scatter(x_vec2, y_vec2, z_vec2)

        
    def drawTriangle(self,pt1,pt2,pt3):
        m = vm.Marker()
        m.type = 11
        m.ns = 'lfd_info'
        m.id = 0
        m.header.frame_id = 'torso_lift_link'
        m.points = [self.gen_utils.vecToRosPoint(pt1), self.gen_utils.vecToRosPoint(pt2), self.gen_utils.vecToRosPoint(pt3)]
        #m.scale.x = 0.20
        #m.scale.y = 0.05
        #m.scale.z = 0.01
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 0.6
        m.lifetime = rospy.Duration(0.0)
        self.rviz_pub.publish(m)
    
     
    def drawGoal(self,goal):
        m = vm.Marker()
        m.type = 0
        m.ns = 'lfd_info'
        m.id = 0
        m.header.frame_id = 'torso_lift_link'
        m.pose = self.gen_utils.vecToRosPose(goal)
        m.scale.x = 0.20
        m.scale.y = 0.05
        m.scale.z = 0.01
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0.0)
        self.rviz_pub.publish(m)
        
        
    # s = size[x,y,z], c=color[r,g,b,a]   
    def drawArrow(self,pos,s,c):
        m = vm.Marker()
        m.type = 0
        m.ns = 'lfd_info'
        m.id = 0
        m.header.frame_id = 'torso_lift_link'
        m.pose = self.gen_utils.vecToRosPose(pos)
        m.scale.x = 0.20
        m.scale.y = 0.05
        m.scale.z = 0.01
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0.0)
        self.rviz_pub.publish(m)


    def drawMarkerGoal(self,goal):
        m = vm.Marker()
        m.type = 1
        m.ns = 'lfd_info'
        m.id = 1
        m.header.frame_id = 'torso_lift_link'
        m.pose = self.gen_utils.vecToRosPose(goal)
        m.scale.x = 0.045
        m.scale.y = 0.045
        m.scale.z = 0.008
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0.0)
        self.rviz_pub.publish(m)
    
    
    def publishMarkerGoal(self,goal):
        self.tf_broad.sendTransform(goal[0:3], goal[3:7], rospy.Time.now(), 'control_marker_goal', 'torso_lift_link') 

    
    def drawScrewHole(self):
        m = vm.Marker()
        m.type = 2
        m.ns = 'lfd_info'
        m.id = 2
        m.header.frame_id = 'ar_marker_8'
        m.pose.position.x = -0.12
        m.pose.position.y = -0.115
        m.pose.position.z = 0.0
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 0.0
        m.scale.x = 0.01
        m.scale.y = 0.01
        m.scale.z = 0.01
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0.0)
        self.rviz_pub.publish(m)

    
    #If which_screw = 0, draw real one, if which_screw = 1, draw imagined one relative to control_marker_goal
    def drawLegScrew(self, which_screw):
        m = vm.Marker()
        m.type = 0
        m.ns = 'lfd_info'
        
        if(which_screw == 0):
            m.header.frame_id = 'ar_marker_0'
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.id = 3
        else:
            m.header.frame_id = 'control_marker_goal'
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.id = 4
            
        m.pose.position.x = -0.045
        m.pose.position.y = 0.0
        m.pose.position.z = -0.025
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 1.0
        m.pose.orientation.w = 0.0
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.03
        
        m.lifetime = rospy.Duration(0.0)
        self.rviz_pub.publish(m)


    def drawPath(self,path):
        m = vm.Marker()
        m.type = 7
        m.ns = 'lfd_info' 
        m.id = 5
        m.header.frame_id = 'torso_lift_link'
        npts = len(path)

        i = 0
        for element in path:
            p = geometry_msgs.msg.Point()
            p.x = element[0]  
            p.y = element[1]  
            p.z = element[2] 
            m.points.append(p)    

            c = std_msgs.msg.ColorRGBA()
            c.r = float(2*i) / float(npts)
            if c.r > 1.0:
                c.r = 1.0
            c.g = 1 - c.r
            c.b = 0
            c.a = 1
            m.colors.append(c)
            i += 1

        m.scale.x = 0.01
        m.scale.y = 0.01
        m.scale.z = 0.01
        m.color.r = 1.0
        m.color.g = 0.5
        m.color.b = 0.5
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0.0)
        self.rviz_pub.publish(m)

     
    def drawPlan(self,plan):
        m = vm.Marker()
        m.type = 7
        m.ns = 'lfd_info'
        m.id = 6
        m.header.frame_id = 'torso_lift_link'
        
        npts = len(plan.points)
        for i in xrange(npts):
            p = geometry_msgs.msg.Point()
            c = std_msgs.msg.ColorRGBA()
            p.x = plan.points[i].positions[0]
            p.y = plan.points[i].positions[1]
            p.z = plan.points[i].positions[2]
            c.r = float(2*i) / float(npts)
            if c.r > 1.0:
                c.r = 1.0
            c.g = 1 - c.r
            c.b = 0
            c.a = 1
            m.points.append(p)
            m.colors.append(c)    

        m.scale.x = 0.01
        m.scale.y = 0.01
        m.scale.z = 0.01
        m.color.r = 1.0
        m.color.g = 0.5
        m.color.b = 0.5
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0.0)
        self.rviz_pub.publish(m)

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
import matplotlib.pyplot as plt
import pickle
from pr2_lfd_utils import generalUtils
from pr2_lfd_utils import trajUtils
from pr2_lfd_utils import moveUtils
from pr2_lfd_utils import drawUtils
from pr2_lfd_utils import arWorldModel
from pr2_lfd_utils import skillParse
import sys
import os
import os.path
from pr2_lfd_utils import dmpExec
        



#Calculate the distance from the goal to determine convergence
def isConverged(curr_pos, goal_pos, goal_thresh):
    conv = True
    for i in range(0,7):
        diff = math.fabs(curr_pos[i] - goal_pos[i])
        if diff > goal_thresh[i]: 
            conv = False
    return conv

      
if __name__ == '__main__':
    try:
    
        rospy.init_node('SingleReplayNode')
        #rospy.on_shutdown(shutdown_cb)

        RIGHT_ARM = 0
        LEFT_ARM = 1
   
        #Setup utilities and world model
        gen_utils = generalUtils.GeneralUtils()
        traj_utils = trajUtils.TrajUtils()
        move_utils = moveUtils.MoveUtils()
        draw_utils = drawUtils.DrawUtils()
        wm = arWorldModel.ARWorldModel()

        #Parameters that select a skill to execute and the marker frame it is in (-1 for torso)
        if (len(sys.argv) == 6):
            whicharm = int(sys.argv[1])
            skill_id = int(sys.argv[2])
            control_frame = int(sys.argv[3])
            goal_frame = int(sys.argv[4])
            plan_only = int(sys.argv[5]) 
        else:
            print "\nAborting! Wrong number of command line args"
            print "Usage: python singleReplay <whicharm> <skill_id> <control_frame> <goal_frame> <plan_only>"
            print "0 for right arm, 1 for left arm"
            print "-1 for control frame: wrist_roll_joint ; -1 for goal_frame: torso_lift_link ; otherwise use ID of marker\n"
            sys.exit(0)
        
        #Construct file names
        basename = 'data/bagfiles/alex/'
        demofile = basename + 'demo' + str(skill_id) + '.bag'
        picklefile = basename + 'Pickle' + str(skill_id) + '.txt'
        matfile = basename + 'Mat' + str(skill_id) + '.txt'
        markerfile = basename + 'Marker' + str(skill_id) + '.txt'
       
       
       

        #################### Load data and process trajectories ####################
        
        #Get the traj of the gripper
        arm_control_data = skillParse.singleSkillParse(picklefile)

        #Get the traj of the goal marker, if approprite
        if(goal_frame >= 0):
            marker_goal_data = traj_utils.createMarkerTrajFromFile(markerfile, goal_frame, -1)
        
        #If a control frame other than the gripper is specified, get that marker's trajectory and add on gripper info
        if (control_frame >= 0):
            #Estimate the rigid transform between marker and wrist and infer the rest from arm pose
            traj_data = traj_utils.inferMarkerTrajFromRigidTrans(arm_control_data, markerfile, control_frame)
        #Otherwise, get the whole skill demo from the arm traj
        else:
            traj_data = arm_control_data




        #################### Learn DMP and prepare for replay ####################
        
        dmp_exec = dmpExec.DMPExec()
        (tau, dmp_list) = dmp_exec.learnDMP(traj_data)
        #print "dmp result: ", tau, dmp_list
        gf = goal_frame
        #no_exec = True
        

        
        if(goal_frame >= 0):
            dmp_exec.executeDMP(whicharm, tau, dmp_list, traj_data[0], traj_data[-1], plan_only_no_exec=plan_only, goal_frame=gf, marker_goal=marker_goal_data[0])     
        else:
            dmp_exec.executeDMP(whicharm, tau, dmp_list, traj_data[0], traj_data[-1], plan_only_no_exec=plan_only, goal_frame=gf)     

     
      
           

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
            
            

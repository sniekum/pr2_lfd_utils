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
import numpy as np
import moveUtils
import generalUtils
import trajUtils
import drawUtils
import bag2mat
import skillParse
import recordInteraction
import dmp.srv
import dmp.msg
import arWorldModel
        

class DMPExec:
    
    def __init__(self):
        self.gen_utils = generalUtils.GeneralUtils()
        self.traj_utils = trajUtils.TrajUtils()
        self.move_utils = moveUtils.MoveUtils()
        self.draw_utils = drawUtils.DrawUtils()
        self.wm = arWorldModel.ARWorldModel()
        
    
   
    def makeLFDRequest(self, dims, traj, dt, K_gain, D_gain, num_bases):
        demotraj = dmp.msg.DMPTraj()
        
        for i in range(len(traj)):
            pt = dmp.msg.DMPPoint();
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
            
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims
        
        print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', dmp.srv.LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "LfD done"    
            
        return resp;


    def makeSetActiveRequest(self, dmp_list):
        try:
            sad = rospy.ServiceProxy('set_active_dmp', dmp.srv.SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            

    def makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt, integrate_iter):
        #print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', dmp.srv.GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        #print "DMP planning done"   
            
        return resp;  
        
        
    #Calculate the distance from the goal to determine convergence
    def isConverged(self, curr_pos, goal_pos, goal_thresh):
        conv = True
        for i in range(0,7):
            diff = math.fabs(curr_pos[i] - goal_pos[i])
            if diff > goal_thresh[i]: 
                conv = False
        return conv
        
        
    #Learn the DMP via LfD
    def learnDMP(self,
                 traj_data,
                 dt = 0.1,
                 K_gain = 100.0,
                 D_gain = 2.0*np.sqrt(100.0),
                 num_bases = 200):
                    
        dims = len(traj_data[0])
        resp = self.makeLFDRequest(dims,traj_data,dt,K_gain,D_gain,num_bases)
        return (resp.tau, resp.dmp_list)
    
    
    def executeDMP(self,
                   whicharm,
                   tau,
                   dmp_list,
                   demo_start,
                   demo_goal,
                   dt = 0.1,
                   integrate_iter = 1,
                   seg_length = 1000.0,
                   time_scale = 1.0,
                   plan_goal_thresh = [0.001]*8,
                   goal_thresh = [0.01]*8,
                   plan_rate = 1.0,
                   planning_delay = 0.5,
                   plan_only_no_exec = False,
                   use_head_tracking = False,
                   control_frame = -1,
                   goal_frame = -1,
                   marker_goal = None):
                   
        #Get ready to replay DMP under current conditions
        plan_rate = rospy.Rate(plan_rate)
        t_0 = 0.0
        tau = tau * time_scale
        goal = demo_goal
        
        #Active the desired DMP
        self.makeSetActiveRequest(dmp_list)  

        if use_head_tracking:
            #Use the head to track the tag we want to control, if any
            if control_frame >= 0:
                self.move_utils.commandARHeadTrack(control_frame)
            #Otherwise, track the goal tag, if any
            elif goal_frame >= 0:
                self.move_utils.commandARHeadTrack(goal_frame)
    
        #Replay DMP in new scenario with planning/replanning 
        is_converged = False
        first = True
        total_points_passed = 0

        #Plan and execute DMP segments until convergence
        while( (not is_converged) and (not rospy.is_shutdown()) ):           
        
            #Adjust the dmp goal based on new AR tag position if a goal frame other than torso is specified
            if goal_frame >= 0:
                if not marker_goal==None:
                    goal = self.traj_utils.modifyGoalWithMarker(whicharm, demo_goal, marker_goal, goal_frame)
                else: 
                    print "Goal frame specified without marker goal. Doing nothing!"
            
            ros_now = rospy.Time.now()
            
            #For the first segment, use current data as starting point for planning
            if(first):
                first = False
                t_0 = 0.0
                splice_time = ros_now
                
                (gpos, gpos_dot) = self.move_utils.arm[whicharm].getGripPoseInfo()
                
                if (control_frame >= 0):
                    #self.wm.startEstimateRelativeMarkerPoseIncremental(control_frame)
                    x_0_cart = self.wm.getObjectById(control_frame)
                else:    
                    x_0_cart = self.wm.getArmCartState(whicharm)
                    
                x_0_cart.append(gpos)
                x_dot_0_cart = [0.0]*8

                
                #Fix starting state so we don't have the wrong aliased quaternion relative to the skill demo beginning point
                self.gen_utils.minimizeQuaternionError(x_0_cart, demo_start)

            #Otherwise, set the "current" state to what we expect to see in planning_delay seconds from now
            else:
                splice_time = ros_now + rospy.Duration(planning_delay)
                [delay_ind, delay_joints, delay_grip, delay_cart_vel, delay_grip_vel] = self.move_utils.arm[whicharm].getCartTrajStatsAtTime(splice_time)
                
                if (control_frame >= 0):
                    #TODO: Fix this so that we get the delayed wrist pose and calc the delayed marker pose from it.  Ick!
                    #delay_joints --> delay_pose (wrist) --> delay_marker_pose
                    #Same for velocity
                    x_0_cart = self.wm.getObjectById(control_frame)
                else:
                    delay_pose = self.move_utils.arm[whicharm].jointsToCart(delay_joints)
                
                x_0_cart = delay_pose + [delay_grip]
                x_dot_0_cart = delay_cart_vel + [delay_grip_vel]
                
                #Get a DMP plan starting from our current time plus the delay
                total_points_passed += delay_ind
                t_0 = (total_points_passed * dt) + planning_delay
                

                    
            resp = self.makePlanRequest(x_0_cart, x_dot_0_cart, t_0, goal, plan_goal_thresh, seg_length, tau, dt, integrate_iter)
            plan = resp.plan

            
            #Get gripper data from plan
            gripper_data = []
            for p in plan.points:
                gripper_data.append(p.positions[7])

            #Adjust the plan so that we use the gripper to control the control_frame marker and splice the traj in after the delay time
            draw_goal = goal
            if (control_frame >= 0):
                adj_plan = traj_utils.modifyTrajForControlPoint(whicharm, plan, control_frame, True)
                self.draw_utils.drawMarkerGoal(plan.points[-1].positions)
                self.draw_utils.publishMarkerGoal(plan.points[-1].positions)
                self.draw_utils.drawLegScrew(1)
                draw_goal = adj_plan[-1]

            #Draw an arrow in rviz showing the position and orientation of the gripper goal, screw, and hole  
            self.draw_utils.drawGoal(draw_goal)
            #self.draw_utils.drawScrewHole()
            #self.draw_utils.drawLegScrew(0)
            self.draw_utils.drawPlan(plan)
            
            if plan_only_no_exec:
                first = True
                #print
                continue



            #Execute the cartesian plan
            if(control_frame >= 0):            
                self.move_utils.arm[whicharm].followCartTraj(adj_plan, gripper_data, dt, splice_time, True)
            else:
                self.move_utils.arm[whicharm].followCartTrajPlan(plan, gripper_data, dt, splice_time, True)

            #is_converged = isConverged(self.wm.getArmCartState(whicharm), goal, goal_thresh)
            is_converged = True

            print "DMP Executed \n"
            plan_rate.sleep()
            
            
if __name__ == '__main__':   
    rospy.init_node('dmpExecNode')
    
    de = DMPExec() 
    demo_num = 1
    whicharm = 1
    
    #traj_data = de.getTrajFromBag("../scripts/data/bagfiles/", demo_num, whicharm)
    (tau, dmp_list) = de.learnDMP(traj_data)
    de.executeDMP(tau, dmp_list, whicharm)
    
    

        

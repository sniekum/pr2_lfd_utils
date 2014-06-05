#!/usr/bin/env python
#import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy
import actionlib as al  
from pr2_controllers_msgs.msg import * 
from trajectory_msgs.msg import *
#import kinematics_msgs.srv 
#import arm_navigation_msgs.srv
import numpy as np 
import numpy.linalg as la
import matplotlib.pyplot as plt
#from pr2_gripper_traj_action.msg import *
import pickle
import math

from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest, GetPositionIK, GetPositionIKRequest


class CartesianTrajExecIK():
    
    #0=right, 1=left
    def __init__(self, whicharm):
        #Joint velocity limits in rad/sec
        self.vel_limits = [0.8, 0.8, 2.0, 2.0, 3.0, 3.0, 10.0]
        #self.vel_limits = [1.0]*7

        #Set up right/left arm variables
        ik_serv_name = '/compute_ik'
        fk_serv_name = '/compute_fk'

        if(whicharm == 0):
            joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
            link_name = 'r_wrist_roll_link'
            traj_client_name = '/r_arm_controller/joint_trajectory_action'
            gripper_traj_serv_name = '/r_gripper_traj_action'
        else:
            joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
            link_name = 'l_wrist_roll_link'
            traj_client_name = '/l_arm_controller/joint_trajectory_action'
            gripper_traj_serv_name = '/l_gripper_traj_action'
        
        #There must be a planning scene or FK / IK crashes 
        #print 'Waiting for set planning scene service...'
        #rospy.wait_for_service('/environment_server/set_planning_scene_diff')
        #setPlan = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', arm_navigation_msgs.srv.SetPlanningSceneDiff)
        #req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
        #setPlan(req)
        #print 'OK' 
        
        #Connect to the joint trajectory action server
        self.traj_client = al.SimpleActionClient(traj_client_name, JointTrajectoryAction)
        while not self.traj_client.wait_for_server(rospy.Duration(5.0)):
            print "Waiting for the joint_trajectory_action server..."
        print "OK"
        self.traj_goal = JointTrajectoryGoal()
        self.traj_goal.trajectory.points = []
        
        #Connect to the gripper trajectory action server
        #self.gripper_traj_client = al.SimpleActionClient(gripper_traj_serv_name, Pr2GripperTrajAction)
        #while not self.gripper_traj_client.wait_for_server(rospy.Duration(5.0)):
        #    print "Waiting for the gripper traj action server..."
        #print "Connected to gripper traj action server"
        #self.grip_traj_goal = Pr2GripperTrajGoal()
        #self.grip_traj_goal.gripper_traj = []

        #Set up IK / FK service handles
        print 'Waiting for IK service...'
        rospy.wait_for_service(ik_serv_name)
        self.getPosIK = rospy.ServiceProxy(ik_serv_name, GetPositionIK, persistent=True)
        print 'OK'
        print 'Waiting for FK service...'
        rospy.wait_for_service(fk_serv_name)
        self.getPosFK = rospy.ServiceProxy(fk_serv_name, GetPositionFK, persistent=True)
        print "OK"
        
        #Get joint info and set up the constant parts of the IK request and goal
        self.traj_goal.trajectory.joint_names = joint_names
        self.pos_ik_req = GetPositionIKRequest()
        self.pos_ik_req.ik_request.robot_state.joint_state.position = [0]*7
        self.pos_ik_req.ik_request.robot_state.joint_state.name = joint_names
        #self.pos_ik_req.timeout = rospy.Duration(5.0)
        if(whicharm == 0):
          self.pos_ik_req.ik_request.group_name = "right_arm"
        else:
          self.pos_ik_req.ik_request.group_name = "left_arm"

        self.pos_ik_req.ik_request.ik_link_name = link_name
        self.pos_ik_req.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
        
        #Set up the constant parts of the FK request
        self.FKreq = GetPositionFKRequest()
        self.FKreq.header.frame_id = "torso_lift_link"
        self.FKreq.fk_link_names = [link_name]
        self.FKreq.robot_state.joint_state.name = joint_names
      
        #Set up empty logging file
        logfile = open('logIK.pickle', 'w')
        logfile.close()
        
        #Keep track of the correspondence between old points passed in and new respaced points
        self.adjusted_ind = []    
        
        #Keep track if there is a previous traj we were following
        self.is_first_traj = True
        
    
    
    #Send an IK request to the appropriate IK service  
    def makeIKRequest(self, cart_pos, start_angles):
        self.pos_ik_req.ik_request.pose_stamped.pose.position.x = cart_pos[0]
        self.pos_ik_req.ik_request.pose_stamped.pose.position.y = cart_pos[1]
        self.pos_ik_req.ik_request.pose_stamped.pose.position.z = cart_pos[2]
        self.pos_ik_req.ik_request.pose_stamped.pose.orientation.x = cart_pos[3]
        self.pos_ik_req.ik_request.pose_stamped.pose.orientation.y = cart_pos[4]
        self.pos_ik_req.ik_request.pose_stamped.pose.orientation.z = cart_pos[5]
        self.pos_ik_req.ik_request.pose_stamped.pose.orientation.w = cart_pos[6]
        self.pos_ik_req.ik_request.ik_seed_state.joint_state.position = start_angles
    
        response = self.getPosIK(self.pos_ik_req)
        
        #Log the IK request / response
        #logfile = open('logIK.pickle', 'a')
        #pickle.dump([self.pos_ik_req.ik_request,response], logfile)
        #logfile.close()
        
        return response
        
        
        
    def makeFKRequest(self,angles):
        self.FKreq.robot_state.joint_state.position =  angles
        try:
            response = self.getPosFK(self.FKreq)
        except rospy.ServiceException, e:
            print self.FKreq
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
    


    #Use matplotlib to plot the positions in the plan
    def plotPositions(self,plan):
        n_pts = len(plan.points)
        dims = len(plan.points[0].positions)
         
        t_vec = [0]*n_pts
        x_vec = [0]*n_pts
        v_vec = [0]*n_pts
        
        plt.figure()
        for i in range(0,7):
            for j in range(n_pts):
                t_vec[j] = plan.points[j].time_from_start.to_sec()
                x_vec[j] = plan.points[j].positions[i]
            plt.plot(t_vec, x_vec) 
              
  
  
    #Takes a trjectory point and modifies the joint positions so that rotation happens in closest direction to the prev point
    def fixJointRollDirection(self, new_p, last_p):
        for i in [4,6]:
            while(new_p[i] - last_p[i] >= 3.14159):
                new_p[i] -= 6.28318
            while(new_p[i] - last_p[i] < -3.14159):
                new_p[i] += 6.28318
          
          
          
    #If transition over dt is within joint velocity limits, then just return the point to be added to the arm and gripper trajs
    #Otherwise, return multiple points with dt spacing that prevent velocity limit from being exceeded    
    def modifyForJointLimits(self, arm1, arm2, grip1, grip2, dt):

        t_diff = arm2.time_from_start.to_sec() - arm1.time_from_start.to_sec()     
        p1 = arm1.positions
        p2 = arm2.positions   

        new_arm_points = []
        new_grip_points = [] 

        #First, find the joint that has the worst diff to limit ratio, since it will take the most spacing
        diffs = [p2[d]-p1[d] for d in range(0,7)]    
        ratios = [math.fabs(diffs[d] / t_diff) / self.vel_limits[d] for d in range(0,7)]
        max_r = max(ratios)
        
        #Respace points if any joint violates its velocity limit
        if(max_r > 1.0):
            #print "MAX RATIO: ", max_r
            #print arm1.time_from_start.to_sec(), t_diff, diffs
            n_pts = int(math.ceil(max_r))
            increments = [(diffs[d]/n_pts)*dt for d in range(0,7)]
            grip_increment = (grip2-grip1)/n_pts

            #Create a point for each increment that keeps within vel limts
            for i in range(n_pts):
                jp = JointTrajectoryPoint()
                jp.time_from_start = rospy.Duration(arm1.time_from_start.to_sec() + (dt*(i+1)))
                jp.positions = [p1[d] + ((i+1)*increments[d]) for d in range(0,7)]
                new_arm_points.append(jp)
                new_grip_points.append(grip1 + (grip_increment*(i+1)))

        #Otherwise, just return the original points
        else:
            new_arm_points = [arm2]
            new_grip_points = [grip2]

        return [new_arm_points, new_grip_points]
                
    
    
    #Returns the index of the point in the current respaced trajectory that will be active (i.e. robot heading towards) at rostime time 
    def getAdjustedTrajPointAtTime(self,time):
        base_time = self.traj_goal.trajectory.header.stamp
        points = self.traj_goal.trajectory.points
        ind = len(points)-1
        
        for i in range(len(points)):
            if (base_time + points[i].time_from_start) > time:
                ind = i
                break
                
        return ind
        
    
    
    #Returns the index of the point from the original passed-in trajectory that corresponds to the active point in the respaced traj
    def getOrigTrajPointAtTime(self,time):
        ind = self.getAdjustedTrajPointAtTime(time)
        return self.adjusted_ind[ind]
    
    
    
    #Returns the predicted joint pose plus gripper at rostime time
    def getPredictedJointPoseAtTime(self,time):
        ind = self.getAdjustedTrajPointAtTime(time)
        if ind >= 0: 
            if ind >= len(self.grip_traj_goal.gripper_traj):
                ind = len(self.grip_traj_goal.gripper_traj)-1
            if ind >= len(self.traj_goal.trajectory.points) and len(self.traj_goal.trajectory.points) < len(self.grip_traj_goal.gripper_traj):
                ind = len(self.traj_goal.trajectory.points)-1
            return [self.traj_goal.trajectory.points[ind].positions, self.grip_traj_goal.gripper_traj[ind]]
        else:
            return []
    
    
    #Returns the predicted cartesian plus gripper velocity at rostime time
    def getPredictedCartVelAtTime(self,time):
        ind = self.getAdjustedTrajPointAtTime(time)
        next_ind = ind+1
        if next_ind >= len(self.traj_goal.trajectory.points):
            next_ind = ind
        
        p1 = self.traj_goal.trajectory.points[ind].positions
        p2 = self.traj_goal.trajectory.points[next_ind].positions
        c1 = self.makeFKRequest(p1)
        c2 = self.makeFKRequest(p2)
        
        g1 = self.grip_traj_goal.gripper_traj[ind]
        g2 = self.grip_traj_goal.gripper_traj[next_ind]
        
        t1 = self.traj_goal.trajectory.points[ind].time_from_start
        t2 = self.traj_goal.trajectory.points[next_ind].time_from_start
        t_diff = (t2-t1).to_sec()
        
        if t_diff == 0:
            a_vel = [0.0]*7
            g_vel = 0.0
        else:
            a_vel = [(c2[d]-c1[d])/t_diff for d in range(7)]
            g_vel = (g2 - g1) / t_diff
            
        return [a_vel, g_vel]
    
    
  
    #Converts a cart traj into a smooth joint traj, dealing with IK misses
    #Assumes that we are starting from start_angles for IK seeding and interpolation in case of IK misses at beginning of traj
    def cartTrajToJointTraj(self, start_angles, x_vec, dt):
        points = []
        t = dt  #Never start first point at time zero!
        seed_angles = start_angles[:]
        
        for i in range(len(x_vec)): 
        
            #Make sure quaternion is normalized
            x_vec[i][3:7] = x_vec[i][3:7] / la.norm(x_vec[i][3:7])

            #Get the inverse kinematic solution for joint angles
            resp = self.makeIKRequest(x_vec[i], seed_angles)

            if(resp.error_code.val == 1):
                angles = resp.solution.joint_state.position
                jp = JointTrajectoryPoint()
                jp.positions = list(angles)
                jp.time_from_start = rospy.Duration(t)
               
                #Fix the arm and wrist roll so that it keeps going in the correct direction
                if(len(points) > 0):
                    self.fixJointRollDirection(jp.positions, points[-1].positions)
                else:
                    self.fixJointRollDirection(jp.positions, start_angles)    
                
                points.append(jp)
                seed_angles = angles
                
            else:
                print "IK error", resp.error_code.val, "on point", i
            
            #Always increment t, even for IK misses
            t += dt
            
        return points
    
    
        
    #Interpolate to fill in the IK miss gaps
    #We do this because it makes it easier to know where we are going to be at any given time for re-planning
    #If we gave 2 widely spaced (in time) points to the JTAC, we don't know exactly where it would go between the 2 points    
    def fillInIKMissGaps(self, start_angles, points, dt):
        interp_points = []
        last_time = rospy.Duration(0)
        
        for i in range(len(points)):        
            p = points[i]
            diff = p.time_from_start - last_time
            t_diff = diff.to_sec()
            
            #Fill in if the points are not dt spaced as they should be
            if t_diff > dt:
                n_pts = int(t_diff/dt)
                
                if(len(interp_points)==0):
                    start = start_angles
                else:
                    start = interp_points[-1].positions
                
                for j in range(n_pts):
                    increments = [(p.positions[d] - start[d]) / n_pts for d in range(7)]
                    jp = JointTrajectoryPoint()
                    jp.positions = [start[d] + (increments[d]*(j+1)) for d in range(7)]
                    jp.time_from_start = last_time + rospy.Duration(dt*(j+1))
                    interp_points.append(jp)
            
            #Otherwise, just add the point
            else:
                interp_points.append(p)
                
            last_time = p.time_from_start
                
        return interp_points
                   
        
  
    #Follows a cartesian trajectory and tries to synchronize the gripper trajectory with it       
    def followCartTraj(self, x_vec, grip_traj, dt, start_angles, splice_time, blocking):   
        
        total_extra_time = 0
        new_arm_traj = []
        new_grip_traj = []
        new_adj_ind = []    #A mapping from the number of the points sent to original points

        #Figure out where the splice is going to happen and what the starting pose will be
        if self.is_first_traj:
            self.is_first_traj = False
            seg_start_joints = start_angles
            seg_start_grip = grip_traj[0]
        else:
            pred = self.getPredictedJointPoseAtTime(splice_time)
            if len(pred) > 0:
                [seg_start_joints, seg_start_grip] = pred
            else:
                seg_start_joints = start_angles
                seg_start_grip = grip_traj[0]
                

        #Convert to a joint trajectory and then interpolate between IK misses
        #After fillInIKMissGaps, the resulting traj should be the same length as the one input to followCartTraj
        #Thus, our old/new point index correspondence is still one-to-one
        j_traj = self.cartTrajToJointTraj(seg_start_joints, x_vec, dt)
        j_traj_interp = self.fillInIKMissGaps(seg_start_joints, j_traj, dt)
        
        #Set up starting conditions for segment
        start_p = JointTrajectoryPoint()
        start_p.time_from_start = rospy.Duration(0.0)
        start_p.positions = seg_start_joints
        start_g = seg_start_grip
        
        #Respace points based on velocity constraints
        for i in range(len(j_traj_interp)):
            
            #Adjust the time of the current traj point based on how many points we've added in along the way
            j_traj_interp[i].time_from_start += rospy.Duration(total_extra_time)
            
            #Respace for veolcity constraints
            [arm_points, grip_points] = self.modifyForJointLimits(start_p, j_traj_interp[i], start_g, grip_traj[i], dt)
            
            #Add the newly spaced points to our new trajectory
            new_arm_traj += arm_points
            new_grip_traj += grip_points
            
            #Record times and correspondences of points to account for repsacing
            #The adjusted ind is the point from the original traj corresponding to the respaced point that is active.
            n_pts = len(arm_points)
            total_extra_time += (n_pts-1) * dt
            new_adj_ind += [i for x in range(n_pts)]
            
            start_p.time_from_start = arm_points[-1].time_from_start
            start_p.positions = arm_points[-1].positions
            start_g = grip_points[-1]
        
        #Make sure we don't splice in the past, or else it will try to go very quickly to a point far into our traj, breaking velocity bounds
        #Also, allow a bit of time for traj processing in the realtime controller
        if splice_time < rospy.Time.now():
            if len(new_arm_traj) > 0:
                print "******** Splice time earlier than current time: ", (splice_time - rospy.Time.now()).to_sec()
                splice_time = rospy.Time.now() 
                
                #Respace between the current pose and the first point so we don't go too fast, since we've passed where we thought we'd be
                #This might cause jerkiness, since the first point might be behind us now, but there's no other solution unless we re-planning
                #This case seems to come up when there are lots of IK misses and not enough actualy traj time for the delay
                start_p = JointTrajectoryPoint()
                start_p.time_from_start = rospy.Duration(0.0)
                start_p.positions = start_angles
                [arm_points, grip_points] = self.modifyForJointLimits(start_p, new_arm_traj[0], new_grip_traj[0], new_grip_traj[0], dt)
                
                #Add on the new points to the front of the traj and update the times for the rest of the points
                extra = len(arm_points) - 1
                extra_t = rospy.Duration(extra*dt)
                for el in new_arm_traj:
                    el.time_from_start += extra_t
                new_arm_traj = arm_points + new_arm_traj[1:]
            
        #Send trajs to joint action controller and gripper traj action controller
        self.traj_goal.trajectory.points = list(new_arm_traj)
        self.traj_goal.trajectory.header.stamp = splice_time #+ rospy.Duration(dt)    #Add in dt so that point we were heading towards isn't deleted, since it isn't in new plan
        self.traj_client.send_goal(self.traj_goal)
        
        self.grip_traj_goal.gripper_traj = list(new_grip_traj)
        self.grip_traj_goal.dt = dt
        print "printing self.grip_traj_goal: ", self.grip_traj_goal
        self.gripper_traj_client.send_goal(self.grip_traj_goal)
        
        self.adjusted_ind = list(new_adj_ind)
        
        print "Extra time: ", total_extra_time
        #for p in self.traj_goal.trajectory.points:
        #    print p.time_from_start.to_sec(), p.positions
        
        if blocking:
            self.traj_client.wait_for_result()
    
        

if __name__ == '__main__':
    rospy.init_node('cartesianTrajIK')
    
    c_right = CartesianTrajExecIK(0);
    c_left = CartesianTrajExecIK(1);

    #jp1 = JointTrajectoryPoint()
    #jp1.positions = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    #jp1.time_from_start = 2.0

    #jp2 = JointTrajectoryPoint()
    #jp2.positions = [1.1, 2.4, 3.1, 4.6, 6.7, 6.1, 7.1]
    #jp2.time_from_start = 2.5

    #print c_right.modifyForJointLimits(jp1,jp2,0.1,0.4,0.1)

    
    cart_pos = [[0.0522, -0.6312, -0.0145, -0.4855, 0.5246, 0.5617, 0.4165], 
                [0.032, -0.6312, -0.0145, -0.4855, 0.5246, 0.5617, 0.4165],  
                [0.0522, -0.6312, -0.0145, -0.4855, 0.5246, 0.5617, 0.4165], 
                [0.032, -0.6312, -0.0145, -0.4855, 0.5246, 0.5617, 0.4165],
                [0.2562, -0.8812, -0.0245, -0.4855, 0.5246, 0.5617, 0.4165]]
    t_vec = [2.0, 4.0, 6.0, 8.0, 10.0]
    
    cart_pos2 = [[0.032, -0.6312, -0.0145, -0.4855, 0.5246, 0.5617, 0.4165],  
                [0.0522, -0.6312, -0.0145, -0.4855, 0.5246, 0.5617, 0.4165], 
                [0.032, -0.6312, -0.0145, -0.4855, 0.5246, 0.5617, 0.4165],
                [0.2562, -0.8812, -0.0245, -0.4855, 0.5246, 0.5617, 0.4165]]
    t_vec2 = [2.0, 4.0, 6.0, 8.0]
    
    start_angles = [0.2562, -0.8812, -0.0245, -0.4855, 0.5246, 0.5617, 0.4165]
    resp = c_right.followCartTraj(cart_pos, t_vec, start_angles, rospy.Time.now())
    resp = c_right.followCartTraj(cart_pos2, t_vec2, start_angles, rospy.Time.now())# + rospy.Duration(2.0))   
    
    #cart_pos = [[0.0522, 0.6312, -0.0145, -0.4855, 0.5246, 0.5617, 0.4165], [0,0,0,0,0,0,0], [0.2562, 0.8812, -0.0245, -0.4855, 0.5246, 0.5617, 0.4165]]
    #resp = c_left.followCartTraj(cart_pos, t_vec, start_angles, rospy.Time.now())   


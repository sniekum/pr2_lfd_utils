#!/usr/bin/env python
#import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy
import actionlib as al  
from pr2_controllers_msgs.msg import * 
from control_msgs.msg import *
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
    def __init__(self):
        #Joint velocity limits in rad/sec
        self.vel_limits = [0.8, 0.8, 2.0, 2.0, 3.0, 3.0, 10.0]
        #self.vel_limits = [1.0]*7

        #joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
        joint_names = ['l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_shoulder_lift_joint', 'l_shoulder_pan_joint', 'l_upper_arm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
        traj_client_name = '/l_arm_controller/follow_joint_trajectory'
        
        #Connect to the joint trajectory action server
        self.traj_client = al.SimpleActionClient(traj_client_name, FollowJointTrajectoryAction)
        while not self.traj_client.wait_for_server(rospy.Duration(5.0)):
            print "Waiting for the joint_trajectory_action server..."
        print "OK"
        self.traj_goal = FollowJointTrajectoryGoal()
        self.traj_goal.trajectory.points = []
        
        #Get joint info and set up the constant parts of the IK request and goal
        self.traj_goal.trajectory.joint_names = joint_names
        self.traj_goal.trajectory.header.frame_id = '/odom_combined'
    
    
        
  
    #Follows a cartesian trajectory and tries to synchronize the gripper trajectory with it       
    def followCartTraj(self):   
        
        total_extra_time = 0
        new_arm_traj = []

        #Set up starting conditions for segment
        start_p = JointTrajectoryPoint()
        start_p.time_from_start = rospy.Duration(1.0)
        start_p.positions = [-2.0699368602610657, 1.6400811170658096, -0.020001503167534247, 2.115092892845208, 1.6399548605537972, -1.6800396869745104, 1.3979724912094418]
        #start_p.positions = [2.115, -0.020, 1.640, -2.070, 1.640, -1.680, 1.398]
        #start_p.positions = [-1.7423735787742214, 1.413866821149008, -0.17272373271344568, 1.7125739007985532, 1.4350723890163595, -1.8568568066487348, 1.3076117709830855]
        
        points = []
        points.append(start_p)

        #start_p = JointTrajectoryPoint()
        #start_p.time_from_start = rospy.Duration(1.5)
        #start_p.positions = [2.1, -0.010, 1.640, -2.070, 1.640, -1.680, 1.398]
        #points.append(start_p)
    
        #Send trajs to joint action controller and gripper traj action controller
        self.traj_goal.trajectory.points = list(points)
        self.traj_goal.trajectory.header.stamp = rospy.Time.now() #+ rospy.Duration(dt)    #Add in dt so that point we were heading towards isn't deleted, since it isn't in new plan
        self.traj_client.send_goal(self.traj_goal)

        print "Trajectory goal sent to the joint trajectory action server", self.traj_goal
        
        

if __name__ == '__main__':
    rospy.init_node('followtest')
    
    c_left = CartesianTrajExecIK();

    resp = c_left.followCartTraj()
    

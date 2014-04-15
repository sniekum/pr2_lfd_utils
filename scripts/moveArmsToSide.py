#!/usr/bin/python
import roslib
roslib.load_manifest('pr2_lfd_utils')
import rospy
import actionlib as al 
from pr2_controllers_msgs.msg import * 
from trajectory_msgs.msg import *

class MoveArmsToSide():

    def __init__(self):
        self.traj_client = []
        self.traj_client.append(al.SimpleActionClient("l_arm_controller/joint_trajectory_action",JointTrajectoryAction))
        while not self.traj_client[0].wait_for_server(rospy.Duration(5.0)):
            print "Waiting for the joint_trajectory_action server..."
        print "Connected to left joint_trajectory_action server"
        
        self.traj_client.append(al.SimpleActionClient("r_arm_controller/joint_trajectory_action",JointTrajectoryAction))
        while not self.traj_client[1].wait_for_server(rospy.Duration(5.0)):
            print "Waiting for the joint_trajectory_action server..."
        print "Connected to right joint_trajectory_action server"
        
        l_joints = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]        
        r_joints = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
        
        self.goal = []
        self.goal.append(JointTrajectoryGoal())
        self.goal[0].trajectory.joint_names = l_joints;
        self.goal.append(JointTrajectoryGoal())
        self.goal[1].trajectory.joint_names = r_joints;

		#Connect to the gripper action servers
        l_gripper_client_name = 'l_gripper_controller/gripper_action'
        self.l_gripper_client = al.SimpleActionClient(l_gripper_client_name, Pr2GripperCommandAction)
        while not self.l_gripper_client.wait_for_server(rospy.Duration(5.0)):
            print "Waiting for the l gripper action server..."
        print "Connected to l gripper action server"
        self.l_grip_goal = Pr2GripperCommandGoal()

        r_gripper_client_name = 'r_gripper_controller/gripper_action'
        self.r_gripper_client = al.SimpleActionClient(r_gripper_client_name, Pr2GripperCommandAction)
        while not self.r_gripper_client.wait_for_server(rospy.Duration(5.0)):
            print "Waiting for the r gripper action server..."
        print "Connected to r gripper action server"
        self.r_grip_goal = Pr2GripperCommandGoal()
        

    def command_grippers(self, position, max_effort):
        self.l_grip_goal.command.position = position
        self.l_grip_goal.command.max_effort = max_effort
        self.l_gripper_client.send_goal(self.l_grip_goal)

        self.r_grip_goal.command.position = position
        self.r_grip_goal.command.max_effort = max_effort
        self.r_gripper_client.send_goal(self.r_grip_goal)


    def moveToJointAngle(self, angles, isRight):
        jp = JointTrajectoryPoint()
        jp.positions = angles
        jp.time_from_start = rospy.Duration(4.0);
        self.goal[isRight].trajectory.points.append(jp)
        self.goal[isRight].trajectory.header.stamp = rospy.Time.now()
        self.traj_client[isRight].send_goal(self.goal[isRight])
        self.traj_client[isRight].wait_for_result()
        self.goal[isRight].trajectory.points = []
        
if __name__ == '__main__':
    try:
        rospy.init_node('MoveArmsToSideNode')
        mover = MoveArmsToSide()
        mover.moveToJointAngle([2.115, -0.020, 1.640, -2.070, 1.640, -1.680, 1.398], 0)
        mover.moveToJointAngle([-2.115, 0.020, -1.640, -2.070, -1.640, -1.680, 1.398], 1)
        mover.command_grippers(0.08, -1)
        print "Done"
        
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
        
        

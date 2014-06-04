#!/usr/bin/env python

# Software License Agreement (BSD License)
#
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
#  * Neither the name of SRI International nor the names of its
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
# Author: Karol Hausman


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from simple_robot_control import gripper

from std_msgs.msg import String

def moveArmsToSide():

  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_arms_to_side_moveit',
                  anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  rightGripper = gripper.Gripper('r')
  leftGripper = gripper.Gripper('l')
  ## This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("left_arm")

  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
  group.clear_pose_targets()
 
  ## Then, we will get the current set of joint values for the group
  group_left_variable_values = group.get_current_joint_values()
  print "============ Left Arm joint values: ", group_left_variable_values

  left_joint_values = [2.115, -0.020, 1.640, -2.070, 1.640, -1.680, 1.398]
  group.set_joint_value_target(left_joint_values)
  plan_left = group.go(None, True)
  
  leftGripper.closeGripper()



  ## Right arm movement
  group = moveit_commander.MoveGroupCommander("right_arm")

  group.clear_pose_targets()
  group_right_variable_values = group.get_current_joint_values()
  print "============ Right Arm joint values: ", group_right_variable_values

  right_joint_values = [-2.115, 0.020, -1.640, -2.070, -1.640, -1.680, 1.398]
  group.set_joint_value_target(right_joint_values)
  plan_right = group.go(None, False)

  rightGripper.closeGripper()

  print "============ STOPPING"


if __name__=='__main__':
  try:
    moveArmsToSide()
  except rospy.ROSInterruptException:
    pass


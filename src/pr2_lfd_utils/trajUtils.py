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
#import threading
import geometry_msgs
import tf
import singleton
import bag2mat
import generalUtils
import arWorldModel
import numpy as np
import skillParse
import glob
import os.path
import pickle
import rosbag
import subprocess
import yaml


class DemoSegment():
    def __init__(self, skill_id, whicharm, traj, goal_frame, control_frame, object_poses):
        self.skill_id = skill_id
        self.whicharm = whicharm
        self.traj = traj                     #indexed as [point_num][dim]
        self.goal_frame = goal_frame
        self.control_frame = control_frame
        self.object_poses = object_poses     #indexed as [obj_id][point_num][dim]
    
    
class Demonstration():
    def __init__(self, segments):
        self.segments = segments


#Singleton class that contains trajectory processing utility functions
class TrajUtils():
    
    __metaclass__ = singleton.Singleton
    
    #0=right, 1=left
    def __init__(self):
        
        self.tformer = tf.TransformerROS(True, rospy.Duration(10.0))
        self.wm = arWorldModel.ARWorldModel()
        self.gen_utils = generalUtils.GeneralUtils()
        
        
    def getFullTrajFromBag(self, 
                           whicharm,
                           basename, 
                           seg_num,
                           desired_hz=10.0, 
                           use_cart=True, 
                           white_thresh=0.001, 
                           is_sim=False,
                           control_frame = -1,
                           goal_frame = -1):
        """
        Given a directory name and segment number, loads the relevant files from that directory to get a trajectory.  
        Assumes the file only contains a single skill and does not need to be segmented.
        
        Args:
            whicharm (int): Which arm is this trajectory for? 0 for right, 1 for left.
            basename (string): The path to examine for demonstration directories
            seg_num (int): This specifies which set of files to load, corresponding to a segment for the left or right arm.
                           Will get trajectory from basename/part<seg_num>.bag
            desired_hz (double): The rate at which to sample from the bagfile
            use_cart (bool): True if we want cartesian data, false for joint data
            white_thresh (double): The threshold of motion below which we count a timestep as being "motionless" and remove that data
            is_sim (bool): Did this data come from simulation or a real robot?
            control_frame (int): Object id that should be controlled by the skill. No entry means the control point is the end effector by default. 
            goal_frame (int): Object ids that the skill goal is relative to. No entry means that the goal is in the torso frame. 
       
        Returns:
            (Demonstration) A demonstration from the appropriate files.
        """
        
        #Construct file names
        demofile = basename + '/part' + str(seg_num) + '.bag'
        picklefile = basename + '/Pickle' + str(seg_num) + '.txt'
        matfile = basename + '/Mat' + str(seg_num) + '.txt'
        markerfile = basename + '/Marker' + str(seg_num) + '.txt'
        
        #Load data from bagfiles and process 
        #Sample the bagfiles at regular intervals, convert to an easier format to use, and write data to files
        b2m = bag2mat.Bag2Mat(whicharm, is_sim)
        b2m.convertToMat(demofile, picklefile, matfile, markerfile, desired_hz, use_cart, white_thresh)

        #Get the traj of the gripper
        arm_control_data = skillParse.singleSkillParse(picklefile)

        #Get the traj of the goal marker, if approprite
        if(goal_frame >= 0):
            marker_goal_data = TrajUtils.createMarkerTrajFromFile(markerfile, goal_frame, -1)
        
        #If a control frame other than the gripper is specified, get that marker's trajectory and add on gripper info
        if (control_frame >= 0):
            #Estimate the rigid transform between marker and wrist and infer the rest of the traj from arm pose
            #This is much more accurate than following a marker traj from perception
            traj_data = self.inferMarkerTrajFromRigidTrans(arm_control_data, markerfile, control_frame)
        #Otherwise, get the whole skill demo from the arm traj
        else:
            traj_data = arm_control_data
            
        #Smooth the gripper data    
        TrajUtils.gripperSmoother(traj_data, 5)
        
        return traj_data
        

    @staticmethod
    def getDemoListFromFiles(basename,
                             task_objects,
                             control_frames,
                             goal_frames):
        """
        Looks for all directories in a given path, intepreting each as a demonstration, and each bag file within the directory as
        a demo segment. Returns a list of Demonstrations that have been processed.
        
        Args:
            basename (string): The path to examine for demonstration directories
            task_objects (list<int>): A list of object ids relevant to the task
            control_frames (dict<int,int>): A dict mapping skill numbers to object ids, corresponding to the object (if any) 
                                            that should be controlled by the skill. No entry means the control point is the end effector by default. 
            goal_frames (dict<int,int>): A dict mapping skill numbers to object ids, corresponding to the object (if any) 
                                          that the skill goal is relative to. No entry means that the goal is in the torso frame. 
        
        Returns:
            demo_list (list<Demonstration>) A list of demonstrations, each corresponding to one of the directories. Each segment in a demo comes from one bag file.
        """
        
        folders = glob.glob(basename + "/*/")
        demo_list = []
        corr_list = []
        armstr = ["Right ", "Left "]
        total_part_ind = 0
        
        #Go through each directory, each of which corresponds to one full task demo
        for fold in folders:
            seg_list = []
            
            #If this is a correction, figure out which part files to ignore
            #is_corr = False
            #if 'corr' in fold:
            #    ind = fold.find('corr') + 4
            #    stripped = fold[ind:].strip('_')
            #    ignore_part = int(stripped[0])
            #    is_corr = True
    
            #Now, loop through all the left arm/right arm alternating bagfiles that make up the demo
            part_num = 1
            while True:
                #Construct file names
                demofile = fold + '/part' + str(part_num) + '.bag'
                picklefile = fold + '/Pickle' + str(part_num) + '.txt'
                markerfile = fold + '/Marker' + str(part_num) + '.txt'
                parsefile = fold + '/Parse' + str(part_num) + '.txt'
                
                #Keep looping until next set of files does not exist
                if not(os.path.isfile(picklefile) and os.path.isfile(markerfile) and os.path.isfile(parsefile)):
                    break
                
                #Figure out which arm the demo is from based on messages recorded
                left_num = 0
                right_num = 0
                
                info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', demofile], stdout=subprocess.PIPE).communicate()[0])
                for topic in info_dict['topics']:
                    if topic['topic'] == '/l_arm_controller/state' or topic['topic'] == '/l_arm_controller_loose/state':
                        left_num += topic['messages']
                    if topic['topic'] == '/r_arm_controller/state' or topic['topic'] == '/r_arm_controller_loose/state':
                        right_num += topic['messages']
                print "L/R:", left_num, right_num
                if left_num > right_num:
                    whicharm = 1
                else:
                    whicharm = 0
    
                #Process the file
                ret = skillParse.parseDataToSegmentList(picklefile, parsefile)
    
                #Each demo part (file) can have multiple segments in it
                seg_num = 0
                for (skill_id, traj_segment) in ret:
                    gf = -1
                    cf = -1
                    
                    #Get the traj of poses of each of the relevant task objects
                    obj_poses = {}
                    for obj_id in task_objects:
                        obj_poses[obj_id] = TrajUtils.createMarkerTrajFromFile(markerfile, obj_id, -1)
                    
                    #If a control frame other than the gripper is specified, get that marker's trajectory and add on gripper info
                    if len(control_frames) > 0:
                        cf = control_frames[total_part_ind][seg_num]
                        #Estimate the rigid transform between marker and wrist and infer the rest of the traj from arm pose
                        #This is much more accurate than following a marker traj from perception
                        traj_data = TrajUtils.inferMarkerTrajFromRigidTrans(traj_segment, markerfile, cf)
                    #Otherwise, get the whole skill demo from the arm traj
                    else:
                        traj_data = traj_segment
                        
                    #Smooth the gripper data    
                    TrajUtils.gripperSmoother(traj_data, 5)
                    
                    #Create the demo segement
                    gmd = []
                    if len(goal_frames) > 0:
                        gf = goal_frames[total_part_ind][seg_num]
                    id_string = armstr[whicharm] + str(skill_id)
                    seg_num += 1
                    
                    seg_list.append(DemoSegment(id_string, whicharm, traj_data, gf, cf, obj_poses))
                
                part_num += 1
                total_part_ind += 1
            
            #if is_corr:
            #    corr_list.append((connect_node, Demonstration(seg_list)))
            #else:
            demo_list.append(Demonstration(seg_list))
        
        return demo_list
        
        
        
    #Gets rid of sharp changes in the gripper set point by ramping instead of jumping to 
    #set point changes to make function approximation more tractable 
    @staticmethod
    def gripperSmoother(traj,radius):
        n_pts = len(traj)
        smoothed = [0]*n_pts
    
        for i in range(n_pts):
            total = 0
    
            #Get the contribution from the points before the current point
            low = i-radius;
            if (low < 0):
                total += (-low) * traj[0][7]  #account for points that aren't present
                low = 0
            for j in range(low,i):
                total += traj[j][7]
            
            #Get the contribution from the points after and including the current point
            high = i+radius;
            if (high > n_pts):
                total += (high-n_pts) * traj[-1][7]  #account for points that aren't present
                high = n_pts
            for j in range(i,high):
                total += traj[j][7]
            
            #Take the average    
            smoothed[i] = total / (2*radius)
        
        #Set to new smoothed values    
        for i in range(n_pts):
            traj[i][7] = smoothed[i]    



    #Gets rid of quaternion aliasing, which causes sharp discontinuities 
    #that are hard to represent with a function approximator
    def quaternionSmoother(self,traj):
        n_pts = len(traj)
        
        for i in range(1,n_pts):
     
            #check distance to next point with no flip
            t1 = np.fabs(traj[i-1][3] - traj[i][3])
            t2 = np.fabs(traj[i-1][4] - traj[i][4])
            t3 = np.fabs(traj[i-1][5] - traj[i][5])
            t4 = np.fabs(traj[i-1][6] - traj[i][6])
            tsum = t1+t2+t3+t4
            
            #check distance to next point with a sign flip
            f1 = np.fabs(traj[i-1][3] + traj[i][3])
            f2 = np.fabs(traj[i-1][4] + traj[i][4])
            f3 = np.fabs(traj[i-1][5] + traj[i][5])
            f4 = np.fabs(traj[i-1][6] + traj[i][6])
            fsum = f1+f2+f3+f4
            
            #choose the representation that causes the least disturbance
            if(fsum < tsum):
                traj[i][3] = -1 * traj[i][3]
                traj[i][4] = -1 * traj[i][4]
                traj[i][5] = -1 * traj[i][5]
                traj[i][6] = -1 * traj[i][6]
    
         
         
    #Smooth sharp changes in joint angles      
    def jointSmoother(self,traj,radius):
        n_pts = len(traj)
        smoothed = [0]*(n_pts)
        
        for dim in range(0,7):
            for i in range(n_pts):
                total = 0
            
                #Get the contribution from the points before the current point
                low = i-radius;
                if (low < 0):
                    low = 0
                for j in range(low,i):
                    total += traj[j][dim]
                    
                #Get the contribution from the points after and including the current point
                high = i+radius;
                if (high > n_pts):
                    high = n_pts
                for j in range(i,high):
                    total += traj[j][dim]
                    
                #Take the average    
                smoothed[i] = total / ((2*radius)+1)
                
            #Set to new smoothed values    
            for i in range(n_pts):   #Don't do the last point, because we want to make sure the goal stays exactly the same
                traj[i][dim] = smoothed[i]
                
     
     
    #For stability, only use the first observed marker pose, then infer rest of poses from a constant rigid transform from end effector
    #NOTE: Consider changing this so that it also uses the final observed marker pose as a goal. 
    #This would rely on perception more, but take into account slippage without needing to determine marker pose while obj is in motion.
    def inferMarkerTrajFromRigidTrans(self, arm_control_data, markerfile, control_frame):
        modified_traj = []
        marker_avg_pose_wrist = self.estimateRelativeMarkerPoseFromTraj(arm_control_data, markerfile, control_frame)
        
        #Modify each point in the plan
        for p in arm_control_data:
            
            #Make a transformation from torso to the current wrist pose
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = 'torso_lift_link'
            m.child_frame_id = 'curr_wrist'
            m.transform = self.gen_utils.vecToRosTransform(p)
            self.tformer.setTransform(m) 
                
            #Figure out where in the torso frame the marker currently is, assuming it has the same pose relative to the wrist as estimated earlier
            try:
                new_marker_torso = self.tformer.transformPose('torso_lift_link',marker_avg_pose_wrist)
                next_goal = self.gen_utils.rosPoseToVec(new_marker_torso.pose) 

                #Add back on the last gripper goal
                next_goal.append(p[7])
                modified_traj.append(next_goal)

            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                print "\ninferMarkerTraj: Couldn't get AR tag frame transform!"
                print "Warning: Not modifying goal!\n"
                    
        return modified_traj
                    
    
    
    #Shift a DMP trajectory to control a marker, rather than the gripper
    #Do this by taking in a DMP traj that regulates marker position and figure out the gripper poses to make it work
    #Use the rigid transform observed between the gripper and marker currently and assume it will stay the same during whole motion
    def modifyTrajForControlPoint(self, whicharm, plan, m_id):
        modified_traj = []
        
        if self.wm.checkIfAllExist([m_id]):
            curr_cart_pos = self.wm.getArmCartState(whicharm)
            target_marker = self.wm.getObjecyById(m_id) 
        
            #Make a transformation from torso to the last observed tag position
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = 'torso_lift_link'
            m.child_frame_id = 'last_marker_pose'
            m.transform = self.gen_utils.vecToRosTransform(target_marker)
            self.tformer.setTransform(m) 
            
            #Copy the last observed gripper pose into a PoseStamped
            last_grip_ps =  geometry_msgs.msg.PoseStamped()
            last_grip_ps.header.frame_id =  'torso_lift_link'
            last_grip_ps.pose = self.gen_utils.vecToRosPose(curr_cart_pos)
            
            try:
                #Figure out the pose of the last observed gripper goal in the last obs marker frame
                tag_relative_goal = self.tformer.transformPose('last_marker_pose',last_grip_ps)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                print "\nmodifyTrajForControlPoint: Couldn't get AR tag frame transform!"
        
            #Modify each point in the plan
            for p in plan.points:
                
                #Make a transformation from torso to the next plan point for the tag
                m = geometry_msgs.msg.TransformStamped()
                m.header.frame_id = 'torso_lift_link'
                m.child_frame_id = 'next_marker_pose'
                m.transform = self.gen_utils.vecToRosTransform(p.positions)
                self.tformer.setTransform(m)
                
                try:
                    #Using the relative pose from before, put it in the same pose relative to the next tag frame, and transform back to torso
                    tag_relative_goal.header.frame_id = 'next_marker_pose'
                    torso_relative_goal = self.tformer.transformPose('torso_lift_link',tag_relative_goal) 
                    next_goal = self.gen_utils.rosPoseToVec(torso_relative_goal.pose) 
    
                    #Add back on the last gripper goal
                    next_goal.append(p.positions[7])
                    modified_traj.append(next_goal)
    
                except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                    print "\nmodifyTrajForControlPoint: Couldn't get AR tag frame transform!"
                    print "Warning: Not modifying goal!\n"
                    
        else: 
            print "modifyTrajForControlPoint: Invalid marker index:",m_id
                    
        return modified_traj    
        
    
    
    #Computes the geometric median of a list of n-dimensional points.
    #In a tie, chooses index tiebreaker if tiebreaker is in the tie, otherwise chooses lowest index
    def computeGeometricMedian(self,points,tiebreaker):
        if len(points) == 0:
            return []
        
        dists = []
        for p in points:
            total_dist = 0
            for q in points:
                diffs = [p[i]-q[i] for i in xrange(len(p))]
                total_dist += sum([x*x for x in diffs]) 
            dists.append(total_dist)
        
        min_dist = min(dists)
        min_inds = [i for i in xrange(len(dists)) if dists[i] == min_dist]
        if tiebreaker in min_inds:
            return points[tiebreaker]
        else:
            return points[min_inds[0]]
         
    
    
    #Performs n-dimensional median filtering on a trajectory of points
    @staticmethod
    def geometricMedianFilter(points,radius):
        filtered = []
        npts = len(points)
        
        start_pad = [points[0]]*radius
        finish_pad = [points[-1]]*radius
        pad_traj = start_pad + points + finish_pad
        
        #Only calc median if the data isn't missing at this point, because we should interpolate instead
        for i in xrange(radius, npts+radius):
            if(len(pad_traj[i]) > 0):
                segment = pad_traj[i-radius : i+radius+1]
                no_missing = [x for x in segment if len(x) > 0]
                # Use the current value of the cell as the tiebreaker, thus favoring to stay the same in a tie
                median = self.computeGeometricMedian(no_missing, no_missing.index(pad_traj[i]))
                filtered.append(median)
            else:
                filtered.append([])
            
        return filtered
    
    
    
    #Fill in all leading / trailing missing data at beginning / end of traj 
    @staticmethod
    def fillEndPoses(traj):
        points = copy.deepcopy(traj)
        
        #First, fill in the beginning
        for i in range(len(points)):
            if(len(points[i]) > 0):
                points[:i] = [points[i]]*i
                break
            
        #Then, fill in the end
        for i in range(-1,-(len(points)+1),-1):
            if(len(points[i]) > 0):
                points[i:] = [points[i]]*(-i)
                break
            
        return points
                
    
    
    #Interpolates to fill in missing sections of data
    #Assumes trimPoses has already been called on the traj of points
    @staticmethod
    def interpolateMissingData(traj):
        i = 0
        points = copy.deepcopy(traj)
        npts = len(points)
        
        #Check if a block of missing data has started
        while(i < npts-1):
            if(len(points[i]) == 0):
                block_start = i
                
                #Keep going until the missing data ends, or we've run out of data points
                while(i < npts-1):
                    i = i+1
                    if(len(points[i]) > 0):
                        break
                
                #Now interpolate between the edges of the missing block of data    
                #NOTE: data has already been trimmed, so we know that the first and last points are not missing data
                dims = len(points[0])
                block_end = i
                block_size = block_end - block_start
                
                for j in xrange(dims):
                    total_diff = points[block_end][j] - points[block_start-1][j]
                    one_diff = total_diff / (block_size + 1)
                
                    for k in xrange(block_size):
                        ind = block_start + k
                        points[ind].append(points[ind-1][j] + one_diff)
                    
            else:
                i = i + 1
                
        return points
        
        
        
    #If the specified marker_id exists in the list of markers (from a single timestep),
    #return its pose, otherwise returns empty list
    @staticmethod
    def filterMarkers(markers,marker_id):
        if marker_id in markers['id']:
            ind = markers['id'].index(marker_id)
            return [markers['pose'][ind], markers['conf'][ind]]
        else:
            return [[],[]]
    
    
    #Write a trajectory (list of points, which are also lists) to file
    @staticmethod
    def writeTrajToFile(traj, filename):
        f = open(filename, 'w')
        for p in traj:
            for x in p:
                f.write("%s " % x)
            f.write("\n")
        f.close()

    
    #Create a marker trajectory from only the specified marker id
    #Use -1 for no median filtering
    @staticmethod
    def createMarkerTrajFromFile(markerfile, marker_id, filter_radius):
        #Load the marker data 
        marker_data = []
        f = open(markerfile, 'r')
        while 1:
            try: marker_data.append(pickle.load(f))  
            except EOFError: break 
        
        #Create a list of marker poses (or missing data) at each timestep     
        marker_poses = [(TrajUtils.filterMarkers(marks_seen,marker_id))[0] for marks_seen in marker_data]
        filled_poses = TrajUtils.fillEndPoses(marker_poses)
        
        if(len(filled_poses)==0):
            print 'createMarkerTraj: marker', marker_id, 'not seen in demo!'
            return []
        
        #Filter if desired
        if(filter_radius > 0):
            filtered_poses = TrajUtils.geometricMedianFilter(filled_poses,filter_radius)
            traj = TrajUtils.interpolateMissingData(filtered_poses)
        else:
            traj = filled_poses
        
        return traj
        
    
    #Only use observations from times that BOTH observations are currently visible
    @staticmethod
    def getNonStalePairs(marker_data, id1, id2):
        p1 = []
        p2 = []
        for markers in marker_data:
            if id1 in markers['id'] and id2 in markers['id']:
                ind1 = markers['id'].index(id1)
                ind2 = markers['id'].index(id2)

                if markers['viz'][ind1] and markers['viz'][ind2]:
                    p1.append(markers['pose'][ind1])
                    p2.append(markers['pose'][ind2])

        print "Reduced", len(marker_data), "points to", len(p1), "non-stale pairs."
        return [p1,p2]
                

    #Create a traj of relative transforms of 2 specified marker ids  
    def createDifferenceTrajFromFile(self, markerfile, marker_id_1, marker_id_2):
        #Load the marker data                                                                                                   
        marker_data = []
        f = open(markerfile, 'r')
        while 1:
            try: marker_data.append(pickle.load(f))
            except EOFError: break

        #Create a paired list of non-stale pairs of marker poses at each timestep                                        
        [filled_poses_1, filled_poses_2] = TrajUtils.getNonStalePairs(marker_data, marker_id_1, marker_id_2)
 
        if(len(filled_poses_1)==0):
            print 'createMarkerTraj: marker', marker_id_1, 'not seen in demo!'
            return []
        if(len(filled_poses_2)==0):
            print 'createMarkerTraj: marker', marker_id_2, 'not seen in demo!'
            return []

        diff = []
        dims = len(filled_poses_1[0])
        avg = [0.0]*dims

        #Calc the difference
        for i in range(len(filled_poses_1)):
            
            #Track average observation
            for j in range(dims):
                avg[j] += filled_poses_1[i][j] + filled_poses_2[i][j]

            #Make a transformation from torso to the first tag
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = 'torso_lift_link'
            m.child_frame_id = 'diff_marker1'
            m.transform = self.gen_utils.vecToRosTransform(filled_poses_1[i])  
            self.tformer.setTransform(m)
        
            #Copy the second tag's pose into a PoseStamped
            marker2_ps =  geometry_msgs.msg.PoseStamped()
            marker2_ps.header.frame_id =  'torso_lift_link'
            marker2_ps.pose = self.gen_utils.vecToRosPose(filled_poses_2[i])
            
            try:
                #Figure out the pose of the 2nd tag in the 1st tag's frame
                relative_transform_ros = self.tformer.transformPose('diff_marker1',marker2_ps)
                relative_transform = self.gen_utils.rosPoseToVec(relative_transform_ros.pose) 
                diff.append(relative_transform)

            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                print "\ncreateDifferenceTrajFromFile: TF transform error!"


        #Calc average                                                                                                           
        for i in range(dims):
            avg[i] /= (2 * len(diff))

        #Add back in the average for rviz drawing purposes so diff isnt drawn at origin
        drawable = []
        for i in range(len(diff)):
            drawable.append([])
            for j in range(dims):
                drawable[i].append(diff[i][j] + avg[j])

        return [diff,drawable]

        
        
    #Shift a goal based on the movement of a marker, ignoring orientation            
    def modifyGoalWithMarkerNoOrientation(self, whicharm, unmodified_goal, orig_marker, m_id): 
        new_goal = unmodified_goal[:]
        
        if self.wm.checkIfAllExist([m_id]):
            curr_cart_pos = self.wm.getArmCartState(whicharm)
            target_marker = self.wm.getObjectById(m_id) 

            #Just modify x, y, and z, but not quaternion
            new_goal[0] += (target_marker[0] - orig_marker[0])
            new_goal[1] += (target_marker[1] - orig_marker[1])
            new_goal[2] += (target_marker[2] - orig_marker[2])
                
        else:
            print "modifyGoalWithMarkerNoOrientation: Invalid marker index:", id_index

        return new_goal
    


    #Shift a goal based on the movement of a marker, including orientation            
    def modifyGoalWithMarker(self, whicharm, unmodified_goal, orig_marker, m_id): 
        new_goal = unmodified_goal[:]
   
        if self.wm.checkIfAllExist([m_id]):
            curr_cart_pos = self.wm.getArmCartState(whicharm)
            target_marker = self.wm.getObjectById(m_id) 
    
            #Make a transformation from torso to the old tag
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = 'torso_lift_link'
            m.child_frame_id = 'old_ar_tag'
            m.transform = self.gen_utils.vecToRosTransform(orig_marker)  
            self.tformer.setTransform(m)
            
            #Make a transformation from torso to the new tag
            m = geometry_msgs.msg.TransformStamped()
            m.header.frame_id = 'torso_lift_link'
            m.child_frame_id = 'new_ar_tag'
            m.transform = self.gen_utils.vecToRosTransform(target_marker)
            self.tformer.setTransform(m) 
            
            #Copy the goal into a PoseStamped
            goal_ps =  geometry_msgs.msg.PoseStamped()
            goal_ps.header.frame_id =  'torso_lift_link'
            goal_ps.pose = self.gen_utils.vecToRosPose(unmodified_goal)
            
            try:
                #First, figure out the pose of the old goal in the old tag frame
                tag_relative_goal = self.tformer.transformPose('old_ar_tag',goal_ps)
                
                #Then, using that pose, put it in the same pose relative to the new tag frame, and transform back to torso
                tag_relative_goal.header.frame_id = 'new_ar_tag'
                torso_relative_goal = self.tformer.transformPose('torso_lift_link',tag_relative_goal) 
                new_goal = self.gen_utils.rosPoseToVec(torso_relative_goal.pose) 

                #Add back on the last gripper goal
                new_goal.append(unmodified_goal[7])

            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                print "\nmodifyGoalWithMarker: Couldn't get AR tag frame transform!"
                print "Warning: Not modifying goal!\n"
                
        else:
            print "modifyGoalWithMarker: Invalid marker index:", m_id
            return -1

        return new_goal
        

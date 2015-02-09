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


#import roslib; roslib.load_manifest('pr2_lfd_utils'); 
import rospy 
#from object_position_publisher.msg import *
#import kinematics_msgs.srv 
#import arm_navigation_msgs.srv
import rosbag
import pprint
import pickle
import numpy as np
import tf
import sys
from pr2_lfd_utils import generalUtils
from pr2_lfd_utils import trajUtils
from pr2_lfd_utils import kinematicsUtils
import subprocess
import yaml


class Bag2Mat:
    
    #0=right, 1=left
    def __init__(self, whicharm, is_sim): 
        self.is_sim = is_sim
        self.gen_utils = generalUtils.GeneralUtils()
        self.traj_utils = trajUtils.TrajUtils()
        self.kinematics_utils = kinematicsUtils.KinematicsUtils()
        
        #Set up right/left arm variables     
        if(whicharm == 0):
            self.gripper_topic_name = '/r_gripper_controller/state'
            self.mann_pos_topic_name = '/r_arm_controller_loose/state'
            self.cont_pos_topic_name = '/r_arm_controller/state'       
        else:
            self.gripper_topic_name = '/l_gripper_controller/state'
            self.mann_pos_topic_name = '/l_arm_controller_loose/state'
            self.cont_pos_topic_name = '/l_arm_controller/state' 
        
        #Fill this in later, once we figure out what we are currently looking at
        self.pos_topic_name = ''

        self.whicharm = whicharm
         
    def getMatrixRowJoint(self, msgs):
        r = list(msgs[self.pos_topic_name].actual.positions)
        return r
    
    
    def getMatrixRowCart(self, msgs):
        #Do forward kinematics on the current joint angles
        r = self.kinematics_utils.getFK(msgs[self.pos_topic_name].actual.positions, self.whicharm, True)
        if (msgs[self.gripper_topic_name] == -1):
          r.append(0)
          r.append(0)
        else:
          r.append(msgs[self.gripper_topic_name].set_point)
          r.append(msgs[self.gripper_topic_name].process_value)
            
        return r

    #pickleFile is for skillParse.py, matFile is for BP-AR-HMM Matlab code
    def convertToMat(self,bagFile,pickleFile,matFile,markerFile,desiredHz,use_cart,white_thresh):
        #First, figure out if this was a recording of mannequin topics or controller topics
        mannequin_num = 0
        controller_num = 0
        info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagFile], stdout=subprocess.PIPE).communicate()[0])
        for topic in info_dict['topics']:
            if topic['topic'] == '/l_arm_controller/state' or topic['topic'] == '/r_arm_controller/state':
                controller_num += topic['messages']
            if topic['topic'] == '/l_arm_controller_loose/state' or topic['topic'] == '/r_arm_controller_loose/state':
                mannequin_num += topic['messages']
        
        if controller_num > mannequin_num:
            self.pos_topic_name = self.cont_pos_topic_name
            print "controller file"
        else:
            self.pos_topic_name = self.mann_pos_topic_name
            print "mannequin file"
            
        topicList = [self.pos_topic_name, self.gripper_topic_name, '/ar_world_model']
       
        print 'Sampling from file',bagFile,'at',desiredHz,'Hz...'

        dt = 1.0/desiredHz
        bag = rosbag.Bag(bagFile)

        currTopicVals = dict(zip(topicList,[-1]*len(topicList)))
        lastTime = 0
        startupWait = 1
        first = 1
        X = []
        tagMsgs = []
        
        #First, sample the messages every dt seconds and build up info
        for topic, msg, t in bag.read_messages(topics=topicList):
            #Let messages come in for the first 0.2 seconds to make sure all topics have a valid message 
            if(startupWait == 1):
                if(first == 1):
                    first = 0
                    lastTime = t.to_sec()
                if(t.to_sec()-lastTime > 0.2):
                    startupWait = 0
                currTopicVals[topic] = msg    
            
            #After that, record the current state of all topics each time dt seconds pass
            else:
                currTopicVals[topic] = msg
                if(t.to_sec()-lastTime > dt):
                    lastTime = t.to_sec()
                    if(use_cart):
                        r = self.getMatrixRowCart(currTopicVals)
                    else:
                        r = self.getMatrixRowJoint(currTopicVals)
                    X.append(r)
                    tagMsgs.append(currTopicVals['/ar_world_model'])
        
        print len(X), 'samples collected'            
        bag.close()
        
        #Then write the info to files
        try:
            print 'Writing files', pickleFile, ',', matFile, '...'
            pf = open(pickleFile, 'w')
            wf = open(matFile,'w')   
            mf = open(markerFile, 'w')
            
            #Write the mat and pickle files with arm/gripper pose
            if(use_cart):
                self.traj_utils.quaternionSmoother(X)
                [X, tagMsgs] = self.removeWhitespace(X, tagMsgs, white_thresh)
                
                for i in range(len(X)):
                    #print X[i][7], X[i][8]
                    pickle.dump(X[i][0:8],pf)  #Don't write gripper process value to file	
		    
                    for j in range(len(X[i])):
                        if(j != 7):            #Don't write gripper set point to the matrix for parsing (only uses process value)
                            wf.write(str(X[i][j])); wf.write(' ')
                    wf.write('\n')
            else:
                for i in range(len(X)):
                    pickle.dump(X[i],pf)
                    for j in range(len(X[i])):
                        wf.write(str(X[i][j])); wf.write(' ')
                    wf.write('\n')
                    
            #Write the file containing the AR tag info
            if(self.is_sim == 0):
                   
                for curr_msg in tagMsgs:
                    id_data = []
                    pose_data = []
                    frame_data = []
                    conf_data = []
                    viz_data = []
 
                    for mark in curr_msg.objects:
                        id_data.append(mark.id)
                        pose_data.append(self.gen_utils.rosPoseToVec(mark.pose.pose))
                        viz_data.append(mark.currently_visible)

                        #THESE ARE HERE ONLY FOR LEGACY COMPATIBILITY REASONS
                        frame_data.append('torso_lift_link')
                        conf_data.append(1)
                        
                    markers_seen = {'id':id_data, 'pose':pose_data, 'frame':frame_data, 'conf':conf_data, 'viz':viz_data}
                    
                    pickle.dump(markers_seen, mf) 
                    
            mf.close()
            pf.close() 
            wf.close()
            print 'Done'
        except IOError as e:
            print "Error writing files:",e
            
    
    #Gets rid of all parts of the trajectory where no motion occurs        
    def removeWhitespace(self,traj,tagMsgs,thresh):
        n_pts = len(traj)
        dims = len(traj[0])
        new_traj = [traj[0]]
        new_tagMsgs = [tagMsgs[0]]
        new_n = 1
      
        for i in range(1,n_pts):
            maxd = 0
            for j in range(dims):
                d = np.fabs(traj[i-1][j] - traj[i][j])
                if(d > maxd):
                    maxd = d
            if(maxd > thresh):
                new_traj.append(traj[i])
                new_tagMsgs.append(tagMsgs[i])
                new_n += 1
                
        print "Reduced", n_pts, "points to", new_n, "points"
        
        return [new_traj, new_tagMsgs]        
                                           
            

if __name__ == '__main__':     
    rospy.init_node('Bag2Mat_node')

    desiredHz = 10.0
    whicharm = 1
    is_sim = 0
    use_cart = 1
    white_thresh = 0.001
    
    basename = '../scripts/data/bagfiles/'
    demofile = basename + '/part' + str(seg_num) + '.bag'
    picklefile = basename + '/Pickle' + str(seg_num) + '.txt'
    matfile = basename + '/Mat' + str(seg_num) + '.txt'
    markerfile = basename + '/Marker' + str(seg_num) + '.txt'
    
    b2m = Bag2Mat(whicharm, is_sim)
    b2m.convertToMat(demofile, picklefile, matfile, markerfile, desiredHz, use_cart, white_thresh)



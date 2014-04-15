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

import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy 
import bag2mat
import glob
import os.path
import subprocess
import yaml

# Script to run bag2mat on all subfolders in a path to generate all the appropriate 
# mat, pickle, and marker files so we can segment the demos and replay the task
if __name__ == '__main__': 
    
    rospy.init_node('ProcessDemosNode')

    desiredHz = 10.0
    whicharm = 1
    is_sim = 0
    use_cart = 1
    white_thresh = -1 #0.001
    basename = './data/bagfiles/stapler2'
    
    folders = glob.glob(basename + "/*/")
    print folders
        
    #Go through each directory, each of which corresponds to one full task demo
    for fold in folders:
        seg_list = []

        #Now, loop through all the left arm/right arm alternating bagfiles that make up the demo
        part_num = 1
        while True:
            #Construct file names
            demofile = fold + '/part' + str(part_num) + '.bag'
            picklefile = fold + '/Pickle' + str(part_num) + '.txt'
            markerfile = fold + '/Marker' + str(part_num) + '.txt'
            matfile = fold + '/Mat' + str(part_num) + '.txt'
            
            #Keep looping until next set of files does not exist
            if not(os.path.isfile(demofile)):
                break
            
            #Figure out which arm the part is from
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
            
            b2m = bag2mat.Bag2Mat(whicharm, is_sim)
            b2m.convertToMat(demofile, picklefile, matfile, markerfile, desiredHz, use_cart, white_thresh)
            part_num += 1

#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_lfd_utils')
import rospy
import actionlib
import numpy as np
import pickle


def chunks(data, labels):
    currStart = 0
    currSkill = labels[0]
    for i in range(1,len(labels)):
        if labels[i] != currSkill: 
            yield (currSkill, data[currStart:i+1])
            currStart = i
            currSkill = labels[i]
    yield (currSkill, data[currStart:])
    

# Returns data with index order:
# Skill num, traj num, point num, DOFs 
def parseDataToSkillList(datafile, parsefile):
    data = []
    f = open(datafile, 'r')
    while 1:
        try: data.append(pickle.load(f))  
        except EOFError: break
        
    f = open(parsefile, 'r')
    labels = [int(el) for el in f.readlines()]
    lSet = list(set(labels))
    lDict = dict([ (lSet[i],i) for i in range(len(lSet)) ])
    skillSeq = []
    
    parsedTrajs = [ [] for i in range(len(lSet)) ]     
    for (skill,traj) in chunks(data, labels):
        skillSeq.append((skill,len(parsedTrajs[lDict[skill]])))
        parsedTrajs[lDict[skill]].append(traj)
    
    #TODO parsedTrajs should be a dict and we should get rid of lDict, but it is implemented this way for legacy reasons
    return (lDict, skillSeq, parsedTrajs)   
    
    
# Makes ordered list of segments, rather than grouping by skill
def parseDataToSegmentList(datafile, parsefile):
    data = []
    f = open(datafile, 'r')
    while 1:
        try: data.append(pickle.load(f))  
        except EOFError: break
        
    f = open(parsefile, 'r')
    labels = [int(el) for el in f.readlines()]
    
    segmentList = []     
    for seg in chunks(data, labels):
        segmentList.append(seg)
    
    return segmentList   



# Parses a file with only one skill in it, thus requiring no parse label file
def singleSkillParse(datafile):
    data = []
    f = open(datafile, 'r')
    while 1:
        try: data.append(pickle.load(f))  
        except EOFError: break
    
    return data
        
        

if __name__ == '__main__':
    (skillMap, trajs) = parseData('data1.txt','parse1.txt')


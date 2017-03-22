#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for RBE 595/CS 525 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import numpy as np
import matplotlib as plt
#### END OF YOUR IMPORTS ####

draw=[]
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize()
    RaveLoadPlugin('build/rrt_plugin')
    rrtmodule = RaveCreateModule(env,'rrt_module')
    ### END INITIALIZING YOUR PLUGIN ###


    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    #set start config
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
    startconfig = [-0.15,0.075,-1.008,-0.11,0,-0.11,0]
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    with env:
        goalconfig = [0.449,-0.201,-0.151,-0.11,0,-0.11,0]

        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        #         weight 1 : 3.17104
        # weight 2 : 2.75674
        # weight 3 : 2.2325
        # weight 4 : 1.78948
        # weight 5 : 1.42903
        # weight 6 : 0.809013
        # weight 7 : 0.593084
        #
        startconfig = robot.GetActiveDOFValues() #initial configuration

        startTime = time.time()

        path= rrtmodule.SendCommand("Goal %f, %f, %f, %f, %f, %f, %f; GoalBias 0.1; Step 0.3; Weights 3.17104, 2.75674, 2.2325, 1.78948, 1.42903, 0.809013, 0.593084 ")
        drawPath(path,robot,[1,0,0])
        RRTTime = time.time()-startTime
        print 'Time taken by RRT algorithm', RRTTime

        # need to Write function to draw the path
        robot.SetActiveDOFValues(startPose)
        path = stringToFloatList(path)
        traj = RaveCreateTrajectory(env,'')
        traj.Init(robot.GetActiveConfigurationSpecification())

        for i in xrange(len(path)):
            traj.Insert(i,path[i])

        planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1)
        print 'Duration of Trajectory =',traj.GetDuration()

        robot.GetController().SetPath(traj)
         ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

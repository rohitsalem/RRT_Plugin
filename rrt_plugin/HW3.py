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
    RaveLoadPlugin('./librrt_plugin.so')
    rrtmodule = RaveCreateModule(env,'rrt_module')
    if not rrtmodule:
        print "Error loading the module"
        exit()
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

        #
        startconfig = robot.GetActiveDOFValues() #initial configuration

        startTime = time.time()
        stepsize=0.3
        goalBias =0.15;

        path= rrtmodule.SendCommand('MyCommand Goal %f, %f, %f, %f, %f, %f, %f; GoalBias %f; Step %f;  '%tuple(goalconfig+[goalBias,stepsize]))
#        n=path.size();
#        for x in range(n):
#            robot.SetActiveDOFValues(path[x]);
#            time.sleep(0.1)
#        drawPath(path,robot,[1,0,0])

#        RRTtime=time.time()-startTime;
#        print RRTtime
#        print path
        z=[0.530317,-0.0936947,-0.558349,0.118796,0,-0.53903,0];
        robot.SetActiveDOFValues(z)



         ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

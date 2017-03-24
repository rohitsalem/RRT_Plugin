#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for RBE 595/CS 525 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import numpy as np
import matplotlib as plt
#### END OF YOUR IMPORTS ####

handles=[]
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

def stringToFloat(path):
    p=path.split('\n')
    for x in range(len(p)):
	  p[x] = p[x].split(',')
	  for i in range(len(p[x])):
		  p[x][i]=float(p[x][i])
    return p

def drawPath(robot,path,color):
	if type(path) is str: path = stringToFloat(path)
	for i in path:
		robot.SetActiveDOFValues(i)
		handles.append(env.plot3(points=robot.GetLinks()[49].GetTransform()[0:3,3],pointsize=0.01,colors=color,drawstyle=1))

        handles.append(env.drawlinestrip(points=robot.GetLinks()[49].GetTransform()[0:3,3],linewidth=0.1,colors=color))

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
        # file=open('test.txt','w')
        # for i in range(1,101,5):
        #     file.write(str(i/100.0) + "\n")
        #     file.flush
        #     print "GoalBias : "
        #     print i/100.0
        #     startconfig = robot.GetActiveDOFValues() #initial configuration
        #
        #
        #     stepsize=0.3;
        #     goalBias =i/100.0;
        #     shortCutSmooth=1;
        #     iterations = 200;
        #     for j in range(10):
        #         startTime = time.time()
        #         path= rrtmodule.SendCommand('MyCommand Goal %f, %f, %f, %f, %f, %f, %f; GoalBias %f; Step %f;shortCutSmooth %d; iterations %d'%tuple(goalconfig+[goalBias,stepsize,shortCutSmooth,iterations]))
        #         #drawPath(robot,path,[1,0,0])
        #         rrt_time=time.time()-startTime;
        #         print "RRT time : "
        #         print  rrt_time
        #         print "Iteration : "
        #         print j
        #         print "\n"
        #         file.write(str(rrt_time) + "\n")
        #         file.flush()

        # file.close()
            #print stringToFloat(path)
        goalBias=0.75
        stepsize=3
        iterations=200
        shortCutSmooth=1
        startTime = time.time()
        path= rrtmodule.SendCommand('MyCommand Goal %f, %f, %f, %f, %f, %f, %f; GoalBias %f; Step %f;shortCutSmooth %d; iterations %d'%tuple(goalconfig+[goalBias,stepsize,shortCutSmooth,iterations]))
        drawPath(robot,path,[1,0,0])
        rrt_time=time.time()-startTime;
        print "RRT time : "
        print  rrt_time


         ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

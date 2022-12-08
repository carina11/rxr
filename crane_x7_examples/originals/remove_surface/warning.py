#! /usr/bin/python3
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import math
import time
from sensor_msgs.msg import PointCloud2, PointCloud
import ros_numpy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
import random
from moveit_msgs.msg import MotionPlanRequest

import os
import sys

cloud = PointCloud()
robotGoal = PointStamped()

def main():
    
    #for i in range(int(input("How many secs for checking?"))):
    while(True):
        #print(arm.get_current_pose())
        pose = arm.get_current_pose()
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        if(x > 0.5 or y > 0.5 or y < -0.5 or z > 0.5):
            print("outside of the boundary")
            #os.system("rosnode kill randomMove")
            arm.stop()
            print("shutdown")
            set_init_pose()
            sys.exit()
        
        else:
            #print("not outside of the boundary")
            pass

        global cloud
        global robotGoal
        #print(robotGoal.point)
        #print(cloud)
        #print(gripper.get_current_joint_values())
        ifclose = False
        for i in cloud.points:
            if(math.sqrt((i.x - x)** 2 + (i.y - y)** 2 + (i.z - z)** 2) < 0.02 and (i.x -x < 0.015) and (i.y - y < 0.015) and (i.z - z < 0.015)):
                #print(math.sqrt((i.x - robotGoal.point.x)**2 + (i.y - robotGoal.point.y)**2 + (i.z - robotGoal.point.z)**2))
                print(math.sqrt((i.x - x)** 2 + (i.y - y)** 2 + (i.z - z)** 2)) 
                #print()
                pass
            if(math.sqrt((i.x - robotGoal.point.x)**2 + (i.y - robotGoal.point.y)**2 + (i.z - robotGoal.point.z)**2) > 0.05 and math.sqrt((i.x - x)** 2 + (i.y - y)** 2 + (i.z - z)** 2) < 0.02 and math.sqrt((i.x - x)** 2 + (i.y - y)** 2 + (i.z - z)** 2) > 0.01 and ifclose == False and gripper.get_current_joint_values()[0] > 0.5):
                print("Too close to the object")
                ifclose = True
                #os.system("rosnode kill randomMove")
                arm.stop()
                print("shutdown")
                set_init_pose()
                #sys.exit()
            #rospy.sleep(1)

        rospy.sleep(0.5)


def set_init_pose():
    arm.set_joint_value_target( [0, -0.1, 0, -2.5, 0, 1.2, 1.57] )
    arm.go()

def callback(data):
    global cloud
    cloud = data

def getGoal(data):
    try:
        global robotGoal
        #print(data)
        point = data.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position
        if(point.x != 0.0 or point.y != 0.0 or point.z != 0.0):
            if(point.x != robotGoal.point.x):
                if(robotGoal.point.x != 0.0):
                    #rospy.sleep(2)
                    pass
                robotGoal.point.x = point.x
                robotGoal.point.y = point.y
                robotGoal.point.z = point.z
                print(robotGoal.point)
    except:
        pass
        






if __name__ == '__main__':
    rospy.init_node("boundary_warning" + sys.argv[1])


    subscribe = rospy.Subscriber("robotpointcloud" + sys.argv[1], PointCloud, callback)

    goal = rospy.Subscriber('/move_group/motion_plan_request/', MotionPlanRequest, getGoal)

    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")

    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    main()
    rospy.spin()
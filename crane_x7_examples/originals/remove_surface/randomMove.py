#! /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import math
import time
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
import random
import sys




import tf

PI = math.pi


def set_pose(x, y, z, rz, ry, rx):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    x,y,z,w = tf.transformations.quaternion_from_euler(rz, ry, rx)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w

    arm.set_start_state_to_current_state()
    arm.set_pose_target(pose)

    print(arm.get_current_pose())

    ret = arm.go()

    return ret

def open_gripper( _open=True ):
    gripper.set_start_state_to_current_state()
    if _open:
        gripper.set_joint_value_target([0.7, 0.7])
    else:
        gripper.set_joint_value_target([0.1, 0.1])
    gripper.go()

def set_init_pose():
    arm.set_joint_value_target( [0, -0.1, 0, -2.5, 0, 1.2, 1.57] )
    arm.go()


    



def main():
    # 初期姿勢
    set_init_pose()
    open_gripper()

    # 正面のものを掴む
    #x = float(input("x: "))
    #y = float(input("y: "))
    #z = float(input("z: "))
    #set_pose(0.1, y, z, PI/2, 0, PI/2)
    #set_pose( x-0.07, y, z, PI/2, 0,  PI/2  )
    #print(robot_xyz)
    #for i in range(int(input("how many times do you wanna repeat?"))):
    for i in range(100):
        x = random.uniform(0.0, 0.5)
        y = random.uniform(-0.5, 0.5)
        z = random.uniform(0.0, 0.5)
        set_pose(x, y, z, PI/2, 0, PI/2)
        if(rospy.is_shutdown() == True):
            sys.exit()
    
    open_gripper(False)

    # 初期姿勢
    set_init_pose()

    # 左前45度に置く
    set_pose( 0.3, -0.3, 0.1, PI/2, 0,  PI/2+PI/4  )
    open_gripper()

    # 初期姿勢
    set_init_pose()

if __name__ == '__main__':
    rospy.init_node("randomMove")
    





    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")

    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    main()
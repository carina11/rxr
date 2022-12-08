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
from geometry_msgs.msg import Point32





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
    #shouldContinue = input("shall we continue")
    point = rospy.wait_for_message('/sphere', Point32)

    tf_listener.waitForTransform('base_link', 'camera_depth_optical_frame0',rospy.Time(0), rospy.Duration(5))
    pos_from_cam = PointStamped()
    pos_from_cam.header.frame_id = 'camera_depth_optical_frame0'
    pos_from_cam.point.x = point.x
    pos_from_cam.point.y = point.y
    pos_from_cam.point.z = point.z
    robotpoint = tf_listener.transformPoint('base_link', pos_from_cam)
    
    #print(robot_xyz)
    print(robotpoint)
    set_pose(robotpoint.point.x - 0.05 , robotpoint.point.y, robotpoint.point.z , PI/2, 0, PI/2)
    open_gripper(False)
    
    set_pose(0.2, 0.0, 0.7, PI/2, 0, PI/2)
    set_pose(0.3, 0, 0.3, PI/2, 0, PI/2)
    open_gripper()
    # 初期姿勢
    #set_init_pose()

    # 左前45度に置く
    #set_pose( 0.3, -0.3, 0.1, PI/2, 0,  PI/2+PI/4  )
    open_gripper()

    # 初期姿勢
    set_init_pose()

if __name__ == '__main__':
    rospy.init_node("move_to_position")





    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    tf_listener = tf.TransformListener()

    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while(True):
        main()
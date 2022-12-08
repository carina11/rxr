#! /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import math
import time
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped




import tf

PI = math.pi

object_xy = [0, 0]
robot_xyz = [0.0, 0.0, 0.0]

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


def callbackYOLO(bbox):
    global object_xy
    object_xy.pop(0)
    object_xy.pop(0)
    object_xy.append(bbox.bounding_boxes[0].xmean)
    object_xy.append(bbox.bounding_boxes[0].ymean)


def callbackPointCloud2(pc2):
    pc = ros_numpy.numpify(pc2)
    height = pc.shape[0]
    width = pc.shape[1]
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(pc['x'], height * width)
    np_points[:, 1] = np.resize(pc['y'], height * width)
    np_points[:, 2] = np.resize(pc['z'], height * width)
    
    #print(len(np_points))
    #x = input("x:")
    #y = input("y:")
    global object_xy
    x = object_xy[0]
    y = object_xy[1]
    print(x,y)
    #print(np_points[width*int(y)+int(x)])
    if(not np.math.isnan(np_points[width * int(y) + int(x)][0])):
        camera_point = np_points[width*int(y)+int(x)]
        tf_listener.waitForTransform( "base_link" , "camera_link", rospy.Time(0), rospy.Duration(5)  )
        pos_from_cam = PointStamped()
        pos_from_cam.header.frame_id = "camera_link"
        pos_from_cam.point.x = camera_point[0]
        pos_from_cam.point.y = camera_point[1]
        pos_from_cam.point.z = camera_point[2]
        pos_trans = tf_listener.transformPoint("base_link", pos_from_cam)
        print(pos_trans.point)
        global robot_xyz
        robot_xyz = pos_trans.point

    



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
    shouldContinue = input("shall we continue")
    global robot_xyz
    print(robot_xyz)
    if(robot_xyz[0] != 0.0):
        set_pose(0.1, robot_xyz[1], robot_xyz[2], PI/2, 0, PI/2)
        set_pose(robot_xyz[0] - 0.07, robot_xyz[1], robot_xyz[2], PI/2, 0, PI/2)
        open_gripper(False)

    # 初期姿勢
        set_init_pose()

    # 左前45度に置く
    set_pose( 0.3, -0.3, 0.1, PI/2, 0,  PI/2+PI/4  )
    open_gripper()

    # 初期姿勢
    set_init_pose()

if __name__ == '__main__':
    rospy.init_node("move_to_position")

    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes,callbackYOLO)

    rospy.Subscriber("/camera/depth/points", PointCloud2, callbackPointCloud2)



    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    tf_listener = tf.TransformListener()

    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    main()
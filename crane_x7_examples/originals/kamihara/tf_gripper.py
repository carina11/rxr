#! /usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String
import tf
import math
import time
import yaml
import pdb
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage
import tf2_ros
from tf2_geometry_msgs import PointStamped

def trans_ClosetToCamera( camx, camy, camz ):
    
    tf_listener = tf.TransformListener()
    pdb.set_trace()
    while True:
        try:
            tf_listener.waitForTransform(  "/camera_depth_optical_frame", "/closest" , rospy.Time(0), rospy.Duration(5)  )
        except :
            print("hoge")

    pos_from_cam = PointStamped()
    pos_from_cam.header.frame_id = "closest"
    pos_from_cam.point.x = camx
    pos_from_cam.point.y = camy
    pos_from_cam.point.z = camz
    pos_trans = tf_listener.transformPoint( "camera_depth_optical_frame", pos_from_cam )

    return pos_trans
    
def trans_CameraTobase( camx, camy, camz ):
    rospy.loginfo("trans_CameraTobase")
    tf_listener = tf.TransformListener()
    tf_listener.waitForTransform( "base_link" , "camera_depth_optical_frame", rospy.Time(0), rospy.Duration(5)  )

    pos_from_cam = PointStamped()
    pos_from_cam.header.frame_id = "camera_depth_optical_frame"
    pos_from_cam.point.x = camx
    pos_from_cam.point.y = camy
    pos_from_cam.point.z = camz
    pos_trans = tf_listener.transformPoint( "base_link", pos_from_cam )

    return pos_trans


def pick_up(x,y,z):
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    #print("Group names:")
    #print(robot.get_group_names())

    #print("Current state:")
    #print(robot.get_current_state())

    # アーム初期ポーズを表示
    #arm_initial_pose = arm.get_current_pose().pose
    #print("Arm initial pose:")
    #print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    # 掴む準備をする
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを開く
    gripper.set_joint_value_target([0.8, 0.8])
    gripper.go()

    
    # 掴みに行く
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z -0.05
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    
    # ハンドを閉じる
    gripper.set_joint_value_target([0.4, 0.4])
    gripper.go()

    # 持ち上げる
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.0
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()							# 実行

    # 移動する
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.2
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # 下ろす
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.2
    target_pose.position.z = 0.13
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを開く
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    # 少しだけハンドを持ち上げる
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.2
    target_pose.position.z = 0.2
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

def callback(msg):
    #rospy.loginfo(len(msg.transforms))
    #rospy.loginfo(msg.transforms[0].child_frame_id)
    if(msg.transforms[0].child_frame_id == "closest"):
        rospy.loginfo("closest get")
        rospy.loginfo(msg.transforms[0])
        #rospy.loginfo("frame_id : %s",msg.transforms[0].header.frame_id)
        #rospy.loginfo("child_frame_id : %s",msg.transforms[0].child_frame_id)
        #p = trans_ClosetToCamera(0.0,0.0,0.0)
        p2 = trans_CameraTobase(0.126315, 0.086278, 0.550041)
        #rospy.loginfo("transform poing p: %lf, %lf, %lf",p.point.x,p.point.y,p.point.z)
        rospy.loginfo("transform poing p2: %lf, %lf, %lf",p2.point.x,p2.point.y,p2.point.z)
        pick_up(p2.point.x,p2.point.y,p2.point.z)

    return


def main():
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    tfBuffer = tf2_ros.Buffer()
    listerner = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("try")
            trans = tfBuffer.lookup_transform("camera_depth_optical_frame", "closest", rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo("try finish")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("WARNING: tf map to base_link not found.")
            rospy.sleep(1)
            continue

    
        pt = PointStamped()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = "closest"
        pt.point.x=0
        pt.point.y=0
        pt.point.z=-0.1
        rospy.loginfo(pt)
        target_pt = tfBuffer.transform(pt, "camera_depth_optical_frame" ,timeout=rospy.Duration(5.0))
        rospy.loginfo(target_pt)
        p2 = trans_CameraTobase(target_pt.point.x, target_pt.point.y,target_pt.point.z)
        rospy.loginfo(p2)
        pick_up(p2.point.x,p2.point.y,p2.point.z)


    #sub = rospy.Subscriber('/tf', TFMessage, callback, queue_size = 1) # if reception->run

    #print("done")
    #rospy.spin()


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

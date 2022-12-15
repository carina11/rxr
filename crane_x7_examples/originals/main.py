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
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import MotionPlanRequest
import random
import os
import sys
import tf
import requests
import shutil
import speech_recognition as sr


from yolov5.detect import run
object_xy = [0, 0]
PI = math.pi
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
    #print(x,y)
    #print(np_points[width*int(y)+int(x)])
    if(not np.math.isnan(np_points[width * int(y) + int(x)][0])):
        camera_point = np_points[width*int(y)+int(x)]
        tf_listener.waitForTransform( "base_link" , "camera_depth_optical_frame0", rospy.Time(0), rospy.Duration(5)  )
        pos_from_cam = PointStamped()
        pos_from_cam.header.frame_id = "camera_depth_optical_frame0"
        pos_from_cam.point.x = camera_point[0]
        pos_from_cam.point.y = camera_point[1]
        pos_from_cam.point.z = camera_point[2]
        pos_trans = tf_listener.transformPoint("base_link", pos_from_cam)
        #print(pos_trans.point)
        global robot_xyz
        #robot_xyz = pos_trans.point
        robot_xyz[0] = pos_trans.point.x
        robot_xyz[1] = pos_trans.point.y
        robot_xyz[2] = pos_trans.point.z

def listen_voice():
    listener = sr.Recognizer()
    try:
        with sr.Microphone() as source:
            print("Listening...")
            voice = listener.listen(source)
            voice_text = listener.recognize_google(voice)
            print(voice_text)
            if "apple" in voice_text or "Apple" in voice_text:
                return "apple", True
            elif "ball" in voice_text or "Ball" in voice_text:
                return "ball", True
            elif "orange" in voice_text or "Orange" in voice_text:
                return "orange", True
            else:
                return voice_text, False
    except:
        print("sorry I could not listen")
        return "failed", False




def main():
    # 初期姿勢
    set_init_pose()
    open_gripper()

    url = 'http://localhost:8080/snapshot?topic=/camera0/color/image_raw'
    file_name = 'obj.png'
    #tmp = 0
    pickup_object = 'ball'
    while(True):
        voice, success_flag = listen_voice()
        if success_flag == True:
            pickup_object = voice
            res = requests.get(url, stream=True)
            if res.status_code == 200:
                with open(file_name, 'wb') as f:
                    shutil.copyfileobj(res.raw, f)
                try:
                    results = run(device='cpu', source=file_name, weights='yolov5s.pt', nosave=True, save_txt=True)
                except:
                    results = []
                for i in range(len(results)):
                    result = results[i][0]
                    label = results[i][1]
                    print(result)
                    print(label)
                    if(pickup_object in label):
                        x = (result[1] + result[3]) * 640
                        y = (result[2] + result[4]) * 480
                        global object_xy
                        object_xy = [int(x), int(y)]
                        rospy.sleep(1)
                        open_gripper()
                        set_pose(robot_xyz[0] + 0.05, robot_xyz[1], 0.05, PI/2, 0, PI/2)
                        open_gripper(False)
                        set_pose(0.2, 0, 0.1, PI/2, 0, PI/2)
                        open_gripper()

        #tmp = int(input('tmp='))
    

    # 初期姿勢
    set_init_pose()

    # 左前45度に置く
    #set_pose( 0.3, -0.3, 0.1, PI/2, 0,  PI/2+PI/4  )
    #open_gripper()

    # 初期姿勢
    #set_init_pose()

if __name__ == '__main__':
    rospy.init_node("pickup")

    rospy.Subscriber("/camera0/depth/points", PointCloud2, callbackPointCloud2)



    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    tf_listener = tf.TransformListener()

    arm.set_max_velocity_scaling_factor(1.0)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    main()
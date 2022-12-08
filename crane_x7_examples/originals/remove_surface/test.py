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

cloud = PointCloud()



def callback(data):
    global cloud
    cloud = data

def getGoal(goalPoint):
    #print(goalPoint.goal.trajectory.points[len(goalPoint.goal.trajectory.points) - 1].positions)
    #print(goalPoint.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position)
    print(arm.get_current_pose())





if __name__ == '__main__':
    rospy.init_node("testtest")
    subscribe = rospy.Subscriber("/robotpointcloud0", PointCloud, callback)
    tf_listener = tf.TransformListener()
    serviceSubscribe = rospy.ServiceProxy('/compute_fk', GetPositionFK)
    goal = rospy.Subscriber('/move_group/motion_plan_request/', MotionPlanRequest, getGoal)
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")

    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    rospy.spin()
    #rospy.spin()
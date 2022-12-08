#!/bin/python3

from numpy.core.fromnumeric import std
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Header
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointCloud
from geometry_msgs.msg import Point32


import tf
import math
import time
import yaml
import sys

def callback(data):
    cloud1 = rospy.wait_for_message('robotpointcloud1', PointCloud)
    cloud2 = rospy.wait_for_message('robotpointcloud2', PointCloud)

    cloud = PointCloud()
    #cloud.points = data.points + cloud1.points + cloud2.points
    cloud.points = data.points + cloud2.points
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'merged_cloud'
    cloud.header = header

    publish.publish(cloud)
    

def main():
    

    pass


rospy.init_node("merge")
subscribe = rospy.Subscriber("robotpointcloud0", PointCloud, callback)
publish = rospy.Publisher("merged_cloud" , PointCloud, queue_size=10)
rospy.spin()
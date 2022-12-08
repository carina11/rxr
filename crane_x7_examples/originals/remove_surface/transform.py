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
	
	#Get PointCloud from the camera
	cloud = PointCloud()
	#Prepare Header
	header = std_msgs.msg.Header()
	header.stamp = rospy.Time.now()
	header.frame_id = "robotpointcloud" + sys.argv[1]
	cloud.header = header

	#Convert each point to robot coordinate
	'''for i in range(len(data.points)):
		tf_listener.waitForTransform( "base_link" , "camera_depth_optical_frame", rospy.Time(0), rospy.Duration(5)  )
		pos_from_cam = PointStamped()
		pos_from_cam.header.frame_id = "camera_depth_optical_frame"
		pos_from_cam.point.x = data.points[i].x
		pos_from_cam.point.y = data.points[i].y
		pos_from_cam.point.z = data.points[i].z
		pos_trans = tf_listener.transformPoint("base_link", pos_from_cam)
		
		point = Point32()
		point.x = pos_trans.point.x
		point.y = pos_trans.point.y
		point.z = pos_trans.point.z
		cloud.points.append(point)'''
	try:
		tf_listener.waitForTransform('base_link', 'camera_depth_optical_frame' + sys.argv[1],rospy.Time(0), rospy.Duration(5))
		data.header.frame_id = 'camera_depth_optical_frame' + sys.argv[1]
		cloud = tf_listener.transformPointCloud('base_link', data)
		cloud.header = header
	
		print("pcl:", len(cloud.points))
		publish.publish(cloud)
	except:
		pass
	
	


rospy.init_node("transform" + sys.argv[1])
tf_listener = tf.TransformListener()
subscribe = rospy.Subscriber("filtered" + sys.argv[1], PointCloud, callback)
publish = rospy.Publisher("robotpointcloud" + sys.argv[1], PointCloud, queue_size=10)
rospy.spin()
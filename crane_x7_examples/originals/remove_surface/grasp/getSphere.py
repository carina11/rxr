#! /usr/bin/python3
from math import nan
import numpy as np
from numpy.core.numeric import NaN
import rospy
import pcl
from sensor_msgs.msg import PointCloud2, PointCloud
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import std_msgs.msg
from geometry_msgs.msg import Point32
import random
import sys

def callback(data):
    #Convert PointCloud2 to pcl
    pc = ros_numpy.numpify(data)
    height = pc.shape[0]
    width = pc.shape[1]
    print(height)
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(pc['x'], height * width)
    np_points[:, 1] = np.resize(pc['y'], height * width)
    np_points[:, 2] = np.resize(pc['z'], height * width)
    
    #Point Cloud from numpy array
    cloud = pcl.PointCloud()
    cloud.from_array(np_points)
    print(cloud.size)

    #Extract points randomly
    #randompoints = range(0, cloud.size, 17)
    
    #cloud = cloud.extract(randompoints, negative=False)

    #Voxel Filter
    '''sor = cloud.make_voxel_grid_filter()
    leaf = 1 * (10**(-10))
    sor.set_leaf_size(leaf, leaf, leaf)
    voxel_filtered = sor.filter()
    print(voxel_filtered.size)
    print(voxel_filtered)'''

    #Passthrough Filter
    #x
    passthrough = cloud.make_passthrough_filter()
    passthrough.set_filter_field_name("x")
    passthrough.set_filter_limits(-2.0, 2.0)
    cloud_x = passthrough.filter()
    print(cloud_x.size)

    #y
    passthrough = cloud_x.make_passthrough_filter()
    passthrough.set_filter_field_name("y")
    passthrough.set_filter_limits(-2.0, 2.0)
    cloud_y = passthrough.filter()
    print(cloud_y.size)

    #z
    passthrough = cloud_y.make_passthrough_filter()
    passthrough.set_filter_field_name("z")
    passthrough.set_filter_limits(-2.0, 2.0)
    cloud_filtered = passthrough.filter()
    print(cloud_filtered.size)

    #surface segmentation
    seg = cloud_filtered.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)
    seg.set_normal_distance_weight(0.01)
    seg.set_max_iterations(100)
    indices, coefficients = seg.segment()
    cloud_plane = cloud_filtered.extract(indices, negative=True)
    print(cloud_plane.size)

    seg_sphere = cloud_plane.make_segmenter_normals(ksearch=50)
    seg_sphere.set_optimize_coefficients(True)
    seg_sphere.set_model_type(pcl.SACMODEL_SPHERE)
    seg_sphere.set_distance_threshold(0.01)
    seg_sphere.set_radius_limits(0.01, 0.05)
    seg_sphere.set_eps_angle(15 /(180/3.14159265))
    seg_sphere.set_max_iterations(1000000)
    indices, coefficients_sphere = seg_sphere.segment()
    cloud_plane = cloud_plane.extract(indices, negative=False)
    point = Point32()
    point.x = coefficients_sphere[0]
    point.y = coefficients_sphere[1]
    point.z = coefficients_sphere[2]
    point_publisher.publish(point)

    #Prepare header
 
    
    

rospy.init_node('getSphere' + sys.argv[1], anonymous=True)
#pcl_publisher = rospy.Publisher("/filtered" + sys.argv[1], PointCloud, queue_size=10)
point_publisher = rospy.Publisher('/sphere', Point32, queue_size=10)
rospy.Subscriber("/camera" + sys.argv[1] + "/depth/points", PointCloud2, callback)
rospy.spin()

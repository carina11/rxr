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
    randompoints = range(0, cloud.size, 17)
    
    cloud = cloud.extract(randompoints, negative=False)

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


    #Prepare header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "filtered" + sys.argv[1]
    publish_content = PointCloud()
    publish_content.header = header
    

    for i in range(cloud_plane.size):
        try:
            if(cloud_plane[i][0] != nan and cloud_plane[i][1] != nan and cloud[i][2] != nan):
                point = Point32()
                point.x = cloud_plane[i][0]
                point.y = cloud_plane[i][1]
                point.z = cloud_plane[i][2]
                publish_content.points.append(point)
        except:
            pass


    pcl_publisher.publish(publish_content)
    

rospy.init_node('remove_surface' + sys.argv[1], anonymous=True)
pcl_publisher = rospy.Publisher("/filtered" + sys.argv[1], PointCloud, queue_size=10)
rospy.Subscriber("/camera" + sys.argv[1] + "/depth/points", PointCloud2, callback)
rospy.spin()

import numpy as np
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import tf
import math
import time
import yaml



def callback(data):
    pc = ros_numpy.numpify(data)
    #print(pc)
    #height = pc.shape[0]
    #width = pc.shape[1]
    #points=np.zeros((height * width,3), dtype=np.float32)
    #points[:,0]=pc['x']
    #points[:,1]=pc['y']
    #points[:,2]=pc['z']
    #p = pcl.PointCloud(np.array(points, dtype=np.float32))
    #print(p)
    height = pc.shape[0]
    width = pc.shape[1]
    print(height)
    np_points = np.zeros((height * width, 3), dtype=np.float32)
    np_points[:, 0] = np.resize(pc['x'], height * width)
    np_points[:, 1] = np.resize(pc['y'], height * width)
    np_points[:, 2] = np.resize(pc['z'], height * width)
    #print(np_points)
    #print("done")
    '''
    for i in np_points:
        if(not np.isnan(i[0]) or not np.isnan(i[1]) or not np.isnan(i[2])):
            print(i)
    '''
    #print(len(np_points))
    x = input("x:")
    y = input("y:")
    #print(np_points[width*int(y)+int(x)])
    camera_point = np_points[width*int(y)+int(x)]
    

    tf_listener.waitForTransform( "base_link" , "camera_depth_optical_frame", rospy.Time(0), rospy.Duration(5)  )
    pos_from_cam = PointStamped()
    pos_from_cam.header.frame_id = "camera_depth_optical_frame"
    pos_from_cam.point.x = camera_point[0]
    pos_from_cam.point.y = camera_point[1]
    pos_from_cam.point.z = camera_point[2]
    pos_trans = tf_listener.transformPoint("base_link", pos_from_cam)
    print(pos_trans.point)
    print()



rospy.init_node('listener', anonymous=True)
#rospy.init_node("move_to_position")
tf_listener = tf.TransformListener()
rospy.Subscriber("/camera/depth/points", PointCloud2, callback)
rospy.spin()
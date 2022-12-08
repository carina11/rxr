import numpy as np
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy

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
    print(np_points[width*int(y)+int(x)])

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/camera/depth/points", PointCloud2, callback)
rospy.spin()

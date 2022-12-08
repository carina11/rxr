import numpy as np
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2
import ros_numpy

def callback(data):
    pc = ros_numpy.numpify(data)
    print(len(pc))
    print(pc)
    height = pc.shape[0]
    width = pc.shape[1]
    print(height)
    #np_points = np.zeros((height * width, 3), dtype=np.float32)
    #np_points[:, 0] = np.resize(pc['x'], height * width)
    #np_points[:, 1] = np.resize(pc['y'], height * width)
    #np_points[:, 2] = np.resize(pc['z'], height * width)
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
    print(pc[int(y)][int(x)])
    #print(len(pc[int(y)]))

rospy.init_node('listener', anonymous=True)
rospy.Subscriber("/camera/depth/image_raw", Image, callback)
rospy.spin()

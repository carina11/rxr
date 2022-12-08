#!/usr/bin/python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
import tf2_ros
import csv
import tf.transformations as T
from tf2_geometry_msgs import PointStamped

file_path="保存先"

if __name__ == '__main__':
    rospy.init_node('get_robot_pose')
    tfBuffer = tf2_ros.Buffer()
    listerner = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("camera_depth_optical_frame", "closest", rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("WARNING: tf map to base_link not found.")
            rospy.sleep(1)
            continue

        translation = [
            trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
        ]
        quaternion_rotation = [
            trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
        ]
        euler_rotation = T.euler_from_quaternion(quaternion_rotation)

        pt = PointStamped()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = "closest"
        pt.point.x=0
        pt.point.y=0
        pt.point.z=-0.05
        rospy.loginfo(pt)
        target_pt = tfBuffer.transform(pt, "camera_depth_optical_frame" ,timeout=rospy.Duration(5.0))

        #with open(file_path, 'a') as f:
        #    writer = csv.writer(f)
        #    writer.writerow([
        #        trans.header.stamp,
        #        translation[0], translation[1], translation[2],
        #        quaternion_rotation[0], quaternion_rotation[1], quaternion_rotation[2], quaternion_rotation[3],
        #        euler_rotation[0], euler_rotation[1], euler_rotation[2]
        #    ])
        rospy.loginfo("%lf, %lf, %lf",translation[0],translation[1],translation[2])
        rospy.loginfo("%lf, %lf, %lf",target_pt.point.x,target_pt.point.y,target_pt.point.z)
        rospy.loginfo("finish")
        # 取得間隔に合わせて調整
        rospy.sleep(0.1)

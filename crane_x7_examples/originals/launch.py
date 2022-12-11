from sys import breakpointhook
import roslaunch
from roslaunch import launch
import rospy
import sys

package = 'crane_x7_examples'

# Choose 1 or 2 from below
#1. Actual calibration
'''ar = roslaunch.core.Node(package, "ar_marker_recognition.py", respawn=True)
camera_calibration = roslaunch.core.Node(package, "camera_calibration.py", respawn=True, args='0.55 0', output="screen")'''
#2. Use tf directly
tf0 = roslaunch.core.Node('tf', 'static_transform_publisher', respawn=True, args="-0.0150 0.2909 0.4428 -2.0631 0.0396 -2.1875 /base_link /camera_depth_optical_frame0 100")
tf1 = roslaunch.core.Node('tf', 'static_transform_publisher', respawn=True, args="2.0453 1.0080 0.3726 -0.8413 3.1612 -4.7239 /base_link /camera_depth_optical_frame1 100")
tf2 = roslaunch.core.Node('tf', 'static_transform_publisher', respawn=True, args='0.0659 -0.5015 0.3515 -0.7891 6.1963 -2.0006 /base_link /camera_depth_optical_frame2 100')

# Remove floor from pcl
#removeSurfaces0 = roslaunch.core.Node(package, 'removeSurfaces.py', respawn=True, args='0')
#removeSurfaces1 = roslaunch.core.Node(package, 'removeSurfaces.py', respawn=True, args='1')
#removeSurfaces2 = roslaunch.core.Node(package, 'removeSurfaces.py', respawn=True, args='2')

# Transform camera coordinate to robot coordinate
#transform0 = roslaunch.core.Node(package, "transform.py", respawn=True, args='0')
#transform1 = roslaunch.core.Node(package, "transform.py", respawn=True, args='1')
#transform2 = roslaunch.core.Node(package, "transform.py", respawn=True, args='2')


# Surveillance node
#warning0 = roslaunch.core.Node(package, "warning.py", respawn=True, args="0", output='screen')
#warning1 = roslaunch.core.Node(package, "warning.py", respawn=True, args="1")
#warning2 = roslaunch.core.Node(package, "warning.py", respawn=True, args="2")


# image streaming
streaming = roslaunch.core.Node('web_video_server', 'web_video_server', respawn=True)

# Lauch 
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

processlist = []
# Choose 1 or 2 from below
# 1. Acutual Calibration
'''
processlist.append(launch.launch(ar))
processlist.append(launch.launch(camera_calibration))'''
#2. Use tf
processlist.append(launch.launch(tf0))
processlist.append(launch.launch(tf1))
processlist.append(launch.launch(tf2))

# Launch each node
'''processlist.append(launch.launch(removeSurfaces0))
processlist.append(launch.launch(removeSurfaces1))
processlist.append(launch.launch(removeSurfaces2))
processlist.append(launch.launch(transform0))
processlist.append(launch.launch(transform1))
processlist.append(launch.launch(transform2))
processlist.append(launch.launch(warning0))'''
processlist.append(launch.launch(streaming))

    
try:
    launch.spin()
finally:
    launch.shutdown()


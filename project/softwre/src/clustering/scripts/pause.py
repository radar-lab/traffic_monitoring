#!/usr/bin/env python
# Author: Feng Jin

import argparse
import os
import time
import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import rosbag
import rospy
import random
from rospy.numpy_msg import numpy_msg
from traffic_monitoring.msg import RadarScan
from clustering_pkg.msg import cluster
from visualization_msgs.msg import Marker
from utilities import marker_color_map
from sklearn import mixture
from sensor_msgs.msg import CompressedImage

def publish_new_marker(cluster_msg):
    # Publish the original results as marker
    # Fullfill the marker
    marker = Marker()
    marker.header.frame_id = "display_markers"
    marker.header.stamp = rospy.get_rostime()
    marker.type = Marker.SPHERE
    marker.id = cluster_msg.point_id

    marker_duration = 100
    
    marker.lifetime        = rospy.Duration.from_sec(marker_duration)
    marker.action          = marker.ADD

    xy_plane_range         = cluster_msg.range * np.cos(cluster_msg.elev)
    marker.pose.position.x = xy_plane_range * np.cos(cluster_msg.angle)
    marker.pose.position.y = -xy_plane_range * np.sin(cluster_msg.angle)
    marker.pose.position.z = cluster_msg.range * np.sin(cluster_msg.elev)

    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 0

    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.a = 1

    if cluster_msg.target_idx < 253:
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 0
    else:
        marker.color.r = 255
        marker.color.g = 255
        marker.color.b = 255
    # Publish the marker
    cluster_marker_pub.publish(marker)
        
if __name__ == '__main__':
    global cluster_marker_pub

    rospy.init_node('display', anonymous=True)
    
    cluster_marker_pub  = rospy.Publisher('/display/display_markers', Marker, queue_size = 100)
    rospy.Subscriber('/traffic_monitoring/radar_scan', RadarScan, publish_new_marker)

    rospy.spin()

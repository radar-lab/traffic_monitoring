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

def cluster_publisher_cb(received_marker):
    received_marker.header.frame_id = "clustering_marker"
    received_marker.lifetime = rospy.Duration.from_sec(marker_duration)
    received_marker.scale.x = 0.3
    received_marker.scale.y = 0.3
    received_marker.scale.z = 0.3
    # Publish the marker
    cluster_publisher.publish(received_marker)

def raw_publisher_cb(received_marker):
    received_marker.header.frame_id = "raw_marker"
    received_marker.lifetime = rospy.Duration.from_sec(marker_duration)
    received_marker.scale.x = 0.3
    received_marker.scale.y = 0.3
    received_marker.scale.z = 0.3
    # Publish the marker
    raw_publisher.publish(received_marker)
        
if __name__ == '__main__':
    global marker_duration
    marker_duration = 10000 # seconds

    rospy.init_node('display', anonymous=True)
    
    cluster_publisher   = rospy.Publisher('/display/cluster_marker', Marker, queue_size = 100)
    raw_publisher       = rospy.Publisher('/display/raw_marker', Marker, queue_size = 100)

    rospy.Subscriber('/clustering/clustering_marker', Marker, cluster_publisher_cb)
    rospy.Subscriber('/clustering/raw_marker', Marker, raw_publisher_cb)
    
    rospy.spin()

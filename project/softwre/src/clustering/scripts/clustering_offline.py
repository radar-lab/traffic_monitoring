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


class read_bagfile:
    def __init__(self):
        self.bagsrcdir  = '~/traffic_monitoring_debug/data/' 
        self.csvdir     = '~/traffic_monitoring_debug/data/'     

    def save_file_to_array(self, filename):
        self.total_frame_data = []
        self.frame_idx  = -1
        self.bag = rosbag.Bag(filename)
        for self.topic, self.msg, self.t in self.bag.read_messages(topics=['/traffic_monitoring/radar_scan']):
            if (self.msg.point_id == 0):
                self.frame_data = []
                self.frame_idx += 1
                self.total_frame_data.append(self.frame_data)
            # -1 is the default target class
            self.frame_data.append([self.frame_idx, self.msg.target_idx, -1, self.msg.point_id, self.msg.range, self.msg.angle, self.msg.elev, self.msg.doppler, self.msg.snr, self.msg.noise])       
        self.bag.close()     
        return self.total_frame_data

    def save_dir_to_array(self, filedir):
        self.total_frame_data = []
        self.frame_idx  = -1
        for self.file in os.listdir(filedir):
            self.bag = rosbag.Bag(os.path.join(filedir,self.file))
            for self.topic, self.msg, self.t in self.bag.read_messages(topics=['/traffic_monitoring/radar_scan']):
                if (self.msg.point_id == 0):
                    self.frame_data = []
                    self.frame_idx += 1
                    self.total_frame_data.append(self.frame_data)
                # -1 is the default target class
                self.frame_data.append([self.frame_idx, self.msg.target_idx, -1, self.msg.point_id, self.msg.range, self.msg.angle, self.msg.elev, self.msg.doppler, self.msg.snr, self.msg.noise])       
            self.bag.close()     
        return self.total_frame_data

class display:
    def __init__(self):
        rospy.init_node('clustering', anonymous=True)
        pass
    def stop_node(self, event):
        rospy.signal_shutdown("All the points have been published.")
        pass
    def publisher_frames(self, data_array, single_frame):
        self.pt_pub      = rospy.Publisher('/clustering/cluster', cluster, queue_size=10)
        self.cluster_marker_pub  = rospy.Publisher('/clustering/clustering_marker', Marker, queue_size = 100)
        self.raw_marker_pub  = rospy.Publisher('/clustering/raw_marker', Marker, queue_size = 100)
        # self.timer       = rospy.Timer(rospy.Duration(5), self.stop_node)
        self.rate = rospy.Rate(1000/frame_period) # 10hz

        if single_frame == True:
            self.data_array = np.array([data_array])
        else:
            self.data_array = np.array(data_array)
        self.frame_idx      = 0
        self.max_frame_idx  = self.data_array.shape[0]
        while not rospy.is_shutdown():
            try:
                for self.frame_idx in range(self.max_frame_idx):
                    self.max_pt_idx = np.array(self.data_array[self.frame_idx]).shape[0]
                    for self.pt_idx in range(self.max_pt_idx):
                        # Fullfill the message
                        self.cluster_msg = cluster()
                        self.cluster_msg.frame_idx      = int(self.data_array[self.frame_idx][self.pt_idx][0])
                        self.cluster_msg.target_idx     = int(self.data_array[self.frame_idx][self.pt_idx][1])
                        self.cluster_msg.target_class   = int(self.data_array[self.frame_idx][self.pt_idx][2])
                        self.cluster_msg.point_id       = int(self.data_array[self.frame_idx][self.pt_idx][3])
                        self.cluster_msg.range          = float(self.data_array[self.frame_idx][self.pt_idx][4])
                        self.cluster_msg.angle          = float(self.data_array[self.frame_idx][self.pt_idx][5])
                        self.cluster_msg.elev           = float(self.data_array[self.frame_idx][self.pt_idx][6])
                        self.cluster_msg.doppler        = float(self.data_array[self.frame_idx][self.pt_idx][7])
                        self.cluster_msg.snr            = int(self.data_array[self.frame_idx][self.pt_idx][8])
                        self.cluster_msg.noise          = int(self.data_array[self.frame_idx][self.pt_idx][9])
                        # Publish this point
                        self.pt_pub.publish(self.cluster_msg)

                        # Publish the results as marker
                        # Fullfill the marker
                        self.marker = Marker()
                        self.marker.header.frame_id = "clustering_marker"
                        self.marker.header.stamp = rospy.get_rostime()
                        self.marker.type = Marker.SPHERE
                        self.marker.id = self.cluster_msg.point_id
                        if single_frame == False:
                            self.marker_duration = float(frame_period)/1000
                        else:
                            self.marker_duration = 1000 # 10 seconds
                        self.marker.lifetime        = rospy.Duration.from_sec(self.marker_duration)
                        self.marker.action          = self.marker.ADD
                        self.xy_plane_range         = self.cluster_msg.range * np.cos(self.cluster_msg.elev)
                        self.marker.pose.position.x = self.xy_plane_range * np.cos(self.cluster_msg.angle)
                        self.marker.pose.position.y = -self.xy_plane_range * np.sin(self.cluster_msg.angle)
                        self.marker.pose.position.z = self.cluster_msg.range * np.sin(self.cluster_msg.elev)
                        self.marker.pose.orientation.x = 0
                        self.marker.pose.orientation.y = 0
                        self.marker.pose.orientation.z = 0
                        self.marker.pose.orientation.w = 0
                        self.marker.scale.x = .1
                        self.marker.scale.y = .1
                        self.marker.scale.z = .1
                        self.marker.color.a = 1
                        self.marker.color.r = marker_color_map[self.cluster_msg.target_class][0]
                        self.marker.color.g = marker_color_map[self.cluster_msg.target_class][1]
                        self.marker.color.b = marker_color_map[self.cluster_msg.target_class][2]
                        # Publish the marker
                        self.cluster_marker_pub.publish(self.marker)

                        # Publish the original results as marker
                        # Fullfill the marker
                        self.marker = Marker()
                        self.marker.header.frame_id = "raw_marker"
                        self.marker.header.stamp = rospy.get_rostime()
                        self.marker.type = Marker.SPHERE
                        self.marker.id = self.cluster_msg.point_id
                        if single_frame == False:
                            self.marker_duration = float(frame_period)/1000
                        else:
                            self.marker_duration = 1000 # 10 seconds
                        self.marker.lifetime        = rospy.Duration.from_sec(self.marker_duration)
                        self.marker.action          = self.marker.ADD
                        self.xy_plane_range         = self.cluster_msg.range * np.cos(self.cluster_msg.elev)
                        self.marker.pose.position.x = self.xy_plane_range * np.cos(self.cluster_msg.angle)
                        self.marker.pose.position.y = -self.xy_plane_range * np.sin(self.cluster_msg.angle)
                        self.marker.pose.position.z = self.cluster_msg.range * np.sin(self.cluster_msg.elev)
                        self.marker.pose.orientation.x = 0
                        self.marker.pose.orientation.y = 0
                        self.marker.pose.orientation.z = 0
                        self.marker.pose.orientation.w = 0
                        self.marker.scale.x = .1
                        self.marker.scale.y = .1
                        self.marker.scale.z = .1
                        self.marker.color.a = 1
                        if self.cluster_msg.target_idx < 253:
                            self.marker.color.r = marker_color_map[self.cluster_msg.target_idx][0]
                            self.marker.color.g = marker_color_map[self.cluster_msg.target_idx][1]
                            self.marker.color.b = marker_color_map[self.cluster_msg.target_idx][2]
                        else:
                            self.marker.color.r = 255
                            self.marker.color.g = 255
                            self.marker.color.b = 255
                        # Publish the marker
                        self.raw_marker_pub.publish(self.marker)

                    self.rate.sleep()
            except rospy.exceptions.ROSException:
                pass 

class new_clustering:
    def __init__(self):
        # Clutter, pedestrian and car
        self.clf = mixture.GaussianMixture(n_components=3, covariance_type='full')

    def cluster_frame(self, frame_data):
        self.frame_data = np.array(frame_data)
        print(self.frame_data.shape)
        self.clf.fit(self.frame_data)
        self.predict_label = self.clf.predict(self.frame_data)
        # Replace the target class with the predicted label
        self.frame_data[:, 2] = self.predict_label
        return self.frame_data

    def fit_GMM(self, training_data):
        print("Total training radar frames: %d" %(len(training_data)))
        training_data_flatten = []
        for frame in training_data:
            training_data_flatten.extend(frame)
        # Use (range, az_angle, el_angle, Doppler, SNR) to fit the GMM
        self.training_data_array = np.array(training_data_flatten)[:, 4:9]
        print("Total training radar points: %d" %(self.training_data_array.shape[0]))
        # Fit the GMM model using the training dataset
        self.clf.fit(self.training_data_array)

    def predict(self, testing_data):
        print("Total testing radar frames: %d" %(len(testing_data)))
        testing_data_flatten = []
        for frame in testing_data:
            testing_data_flatten.extend(frame)
        self.testing_data_array = np.array(testing_data_flatten)
        print("Total testing radar points: %d" %(self.testing_data_array.shape[0]))
        # Predict on the testing dataset
        self.prediction = []
        for frame in testing_data:
            self.testing_data_array = np.array(frame)
            self.radar_measurement = self.testing_data_array[:, 4:9]
            self.predict_label = self.clf.predict(self.radar_measurement)
            # Replace the target class with the predicted label
            self.testing_data_array[:, 2] = self.predict_label
            self.prediction.append(self.testing_data_array)
        return self.prediction
        
if __name__ == '__main__':
    ######################################################################################################
    parser = argparse.ArgumentParser()
    parser.add_argument('--train_filedir', type=str, default=None, help='Load which file. Default: None.')
    parser.add_argument('--test_filedir', type=str, default=None, help='Load which file. Default: None.')
    args = parser.parse_args()

    ######################################################################################################
    # marker_color_map = cm.rainbow(np.linspace(0, 1, num=252))
    # random.shuffle(marker_color_map)
    frame_period = 100 # 100 ms

    ######################################################################################################
    if args.train_filedir is not None:
        print("Reading training file directory...")
        total_training_frame_data = read_bagfile().save_dir_to_array(args.train_filedir)
    if args.test_filedir is not None:
        print("Reading testing file directory...")
        total_testing_frame_data = read_bagfile().save_dir_to_array(args.test_filedir)

    # Show the entire video
    display().publisher_frames(total_training_frame_data, single_frame = False)

    # Show one frame
    # frame_idx = 60
    # display().publisher_frames(total_frame_data[frame_idx], single_frame = True)

    # # Fit GMM and predict one frame data
    # frame_idx = 60
    # predict_frame_data = new_clustering().cluster_frame(total_frame_data[frame_idx])
    # display().publisher_frames(predict_frame_data, single_frame = True)

    # # Fit GMM on all training data and predict all testing data
    # frame_idx = 60
    # model  = new_clustering()
    # model.fit_GMM(total_training_frame_data)
    # prediction_results = model.predict(total_testing_frame_data)
    # display().publisher_frames(prediction_results, single_frame = False)

#!/usr/bin/env python
# Author: Feng Jin

import argparse
import os
import time
import numpy as np
import math
import matplotlib
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
from sklearn.metrics import confusion_matrix, classification_report, precision_recall_curve, average_precision_score
# from sklearn.metrics import jaccard_score
from sklearn.preprocessing import label_binarize
from itertools import cycle
import pickle

font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 22}

matplotlib.rc('font', **font)


class read_bagfile:
    def __init__(self):
        self.bagsrcdir  = '~/traffic_monitoring_debug/data/' 
        self.csvdir     = '~/traffic_monitoring_debug/data/'     

    def save_dir_to_array(self, filedir):
        self.total_frame_data = []
        self.frame_data = []
        self.frame_idx  = 0
        for self.file in os.listdir(filedir):
            # Read the bag file
            self.bag = rosbag.Bag(os.path.join(filedir,self.file))
            # Read all the msg
            for self.topic, self.msg, self.t in self.bag.read_messages(topics=['/traffic_monitoring/radar_scan']):
                # New frame arrives
                if ( (self.msg.point_id == 0) and (self.frame_data) ):
                    self.frame_idx += 1
                    self.total_frame_data.append(self.frame_data)
                    self.frame_data = []
                # If not clutters
                if self.msg.target_idx < 256:
                    # -1 is the default predicted class
                    self.predict_class  = -1
                    # Set ground truth
                    # When we collected the testing data, the pedestrain is on the left (posY<0); 
                    # The car is on the right (posY>=0)
                    # Target_idx greather than 253 is clutter
                    if self.msg.target_idx >= 253:
                        self.target_class = 0 # Clutter in red
                    elif (self.msg.target_idx < 253) and (self.msg.posY < 0):
                        self.target_class = 2 # Pedestrain in green
                    elif (self.msg.target_idx < 253) and (self.msg.posY >= 0):
                        self.target_class = 1 # Car in blue
                    # Grow a frame
                    self.frame_data.append([self.frame_idx, self.msg.target_idx, self.target_class, self.predict_class, \
                        self.msg.posX, self.msg.posY, self.msg.posZ, self.msg.velX,self.msg.velY, self.msg.velZ, \
                        self.msg.point_id, self.msg.range, self.msg.angle, self.msg.elev, self.msg.doppler, self.msg.snr, self.msg.noise])       
            # Close the bag file
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
                        self.cluster_msg.predict_class  = int(self.data_array[self.frame_idx][self.pt_idx][3])
                        self.cluster_msg.point_id       = int(self.data_array[self.frame_idx][self.pt_idx][10])
                        self.cluster_msg.range          = float(self.data_array[self.frame_idx][self.pt_idx][11])
                        self.cluster_msg.angle          = float(self.data_array[self.frame_idx][self.pt_idx][12])
                        self.cluster_msg.elev           = float(self.data_array[self.frame_idx][self.pt_idx][13])
                        self.cluster_msg.doppler        = float(self.data_array[self.frame_idx][self.pt_idx][14])
                        self.cluster_msg.snr            = int(self.data_array[self.frame_idx][self.pt_idx][15])
                        self.cluster_msg.noise          = int(self.data_array[self.frame_idx][self.pt_idx][16])
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
                        self.marker.color.r = marker_color_map[self.cluster_msg.predict_class][0]
                        self.marker.color.g = marker_color_map[self.cluster_msg.predict_class][1]
                        self.marker.color.b = marker_color_map[self.cluster_msg.predict_class][2]
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
                        self.marker.color.r = marker_color_map[self.cluster_msg.target_class][0]
                        self.marker.color.g = marker_color_map[self.cluster_msg.target_class][1]
                        self.marker.color.b = marker_color_map[self.cluster_msg.target_class][2]
                        # if self.cluster_msg.target_idx < 253:
                        #     self.marker.color.r = marker_color_map[self.cluster_msg.target_idx][0]
                        #     self.marker.color.g = marker_color_map[self.cluster_msg.target_idx][1]
                        #     self.marker.color.b = marker_color_map[self.cluster_msg.target_idx][2]
                        # else:
                        #     self.marker.color.r = 255
                        #     self.marker.color.g = 255
                        #     self.marker.color.b = 255
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

    def preprocess(self, data):
        processed_data  = []
        num_points      = 0
        for frame in data:
            processed_frame = []
            num_points = num_points + len(frame)
            for point in frame:
                # Get the point information in Cartesian coord.
                pointR      = point[11]
                pointAZ     = point[12]
                pointEL     = point[13]
                pointX      = pointR*np.cos(pointEL)*np.sin(pointAZ)
                pointY      = pointR*np.cos(pointEL)*np.cos(pointAZ)
                pointZ      = pointR*np.sin(pointEL)
                pointD      = point[14]
                pointSNR    = point[15]
                pointNoise  = point[16]
                # Get the centorid information in Cartesian coord.
                targetX     = point[4]
                targetY     = point[5]
                targetZ     = point[6]
                targetVx    = point[7]
                targetVy    = point[8]
                targetVz    = point[9]
                # Get the point feature vector
                delta_x     = pointX - targetX
                delta_y     = pointY - targetY
                delta_z     = pointZ - targetZ
                delta_D     = pointD - (pointX*targetVx+pointY*targetVy+pointZ*targetVz)/pointR
                pointRCS    = 4*10*np.log10(pointR) + pointSNR*0.1 + pointNoise*0.1
                processed_frame.append([delta_x, delta_y, delta_z, delta_D, pointRCS])
            processed_data.append(processed_frame)
        return len(processed_data), num_points, processed_data 

    def fit_GMM(self, training_data):
        frame_num, point_num, preprocessed_training_data = self.preprocess(training_data)
        print("Total training radar frames: %d" %(frame_num))
        print("Total training radar points: %d" %(point_num))
        assert len(training_data) == len(preprocessed_training_data), "ERROR!"
        # Flatten all the data
        preprocessed_training_data_flatten = []
        for frame in preprocessed_training_data:
            preprocessed_training_data_flatten.extend(frame)
        # Convert to numpy array
        preprocessed_training_data_flatten_array = np.array(preprocessed_training_data_flatten)
        # Fit the GMM model using the training dataset
        self.clf.fit(preprocessed_training_data_flatten_array)

    def predict(self, testing_data):
        frame_num, point_num, preprocessed_testing_data = self.preprocess(testing_data)
        print("Total testing radar frames: %d" %(frame_num))
        print("Total testing radar points: %d" %(point_num))
        assert len(testing_data) == len(preprocessed_testing_data), "ERROR!"
        for frame_idx in range(len(testing_data)):
            testing_data_array = np.array(preprocessed_testing_data[frame_idx])
            # Replace the target class with the predicted label
            prediction_array        = np.array(testing_data[frame_idx])
            prediction_array[:, 3]  = self.clf.predict(testing_data_array)
            testing_data[frame_idx] = list(prediction_array)

    def verify(self, testing_data):
        total_testing_frame_data = []
        for frame in testing_data:
            total_testing_frame_data.extend(frame)
        total_testing_frame_data_array = np.array(total_testing_frame_data)
        print("Total testing data shape is: %s" %(str(total_testing_frame_data_array.shape)))
        # Get the ground truth
        ground_truth    = total_testing_frame_data_array[:, 2]
        # Get the prediction
        prediction      = total_testing_frame_data_array[:, 3]
        # Calculate the confusin matirx
        obj_class = ['clutter', 'car', 'pedestrian']
        print("***********************************************************************************")
        print(classification_report(ground_truth, prediction, target_names=obj_class))
        print("***********************************************************************************")
        print("IoU report:")
        print(obj_class)
        print("***********************************************************************************")
        print(jaccard_score(ground_truth, prediction, average=None))
        self.plot_confusion_matrix(ground_truth, prediction, obj_class, normalize=True)

    def plot_confusion_matrix(self, y_true, y_pred, classes,
                                normalize=True,
                                title=None,
                                cmap=plt.cm.Blues):
        if not title:
            if normalize:
                title = 'Normalized confusion matrix'
            else:
                title = 'Confusion matrix, without normalization'

        # Compute confusion matrix
        cm = confusion_matrix(y_true, y_pred)
        # Only use the labels that appear in the data
        # classes = classes[unique_labels(y_true, y_pred)]
        if normalize:
            cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
            print("Normalized confusion matrix")
        else:
            print('Confusion matrix, without normalization')

        print(cm)

        fig, ax = plt.subplots()
        im = ax.imshow(cm, interpolation='nearest', cmap=cmap)
        ax.figure.colorbar(im, ax=ax)
        # We want to show all ticks...
        ax.set(xticks=np.arange(cm.shape[1]),
            yticks=np.arange(cm.shape[0]),
            # ... and label them with the respective list entries
            xticklabels=classes, yticklabels=classes,
            title=title,
            ylabel='True label',
            xlabel='Predicted label')

        # Rotate the tick labels and set their alignment.
        plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
                rotation_mode="anchor")

        # Loop over data dimensions and create text annotations.
        fmt = '.2f' if normalize else 'd'
        thresh = cm.max() / 2.
        for i in range(cm.shape[0]):
            for j in range(cm.shape[1]):
                ax.text(j, i, format(cm[i, j], fmt),
                        ha="center", va="center",
                        color="white" if cm[i, j] > thresh else "black")
        fig.tight_layout()
        
        plt.show()

def plot_precision_recall(precision, recall, average_precision):
    # setup plot details
    colors = cycle(['navy', 'cornflowerblue', 'darkorange'])

    plt.figure(figsize=(7, 8))
    f_scores = np.linspace(0.2, 0.8, num=4)
    lines = []
    labels = []

    n_classes = 3
    obj_class = ['clutter', 'car', 'pedestrian']
    for i, color in zip(range(n_classes), colors):
        l, = plt.plot(recall[i], precision[i], color=color, lw=2)
        lines.append(l)
        labels.append("Precision-recall for {0} (average precision = {1:0.2f})".format(obj_class[i], average_precision[i]))

    fig = plt.gcf()
    fig.subplots_adjust(bottom=0.25)
    plt.xlim([0.0, 1.0])
    plt.ylim([0.0, 1.05])
    plt.xlabel('Recall')
    plt.ylabel('Precision')
    plt.title('Precision-Recall curve')
    plt.legend(lines, labels, loc=(0, -.38), prop=dict(size=14))
    plt.show()
        
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

    # ######################################################################################################
    # if args.train_filedir is not None:
    #     print("Reading training file directory...")
    #     total_training_data = read_bagfile().save_dir_to_array(args.train_filedir)
    if args.test_filedir is not None:
        print("Reading testing file directory...")
        total_testing_data = read_bagfile().save_dir_to_array(args.test_filedir)

    # # Show the entire video
    # display().publisher_frames(total_training_frame_data, single_frame = False)

    # Show one frame
    # frame_idx = 60
    # display().publisher_frames(total_frame_data[frame_idx], single_frame = True)

    # # Fit GMM and predict one frame data
    # frame_idx = 60
    # predict_frame_data = new_clustering().cluster_frame(total_frame_data[frame_idx])
    # display().publisher_frames(predict_frame_data, single_frame = True)

    # # Fit GMM on all training data and predict all testing data
    # model  = new_clustering()
    # model.fit_GMM(total_training_data)
    # pickle.dump(model, open('model.sav','wb'))

    # Load model and predict
    model = pickle.load(open('model.sav','rb'))
    # model.predict(total_testing_data)

    frame_num, point_num, preprocessed_testing_data = model.preprocess(total_testing_data)
    print("Total testing radar frames: %d" %(frame_num))
    print("Total testing radar points: %d" %(point_num))
    assert len(total_testing_data) == len(preprocessed_testing_data), "ERROR!"

    testing_flatten = []
    for item in total_testing_data:
        testing_flatten.extend(item)
    testing_flatten_array = np.array(testing_flatten)
    ground_truth = testing_flatten_array[:, 2]
    ground_truth_bin = label_binarize(ground_truth, classes=[0, 1, 2])
    print(ground_truth.shape)
    print(ground_truth_bin.shape)

    preprocessed_flatten = []
    for item in preprocessed_testing_data:
        preprocessed_flatten.extend(item)
    preprocessed_flatten_array = np.array(preprocessed_flatten)
    prediction = model.clf.predict_proba(preprocessed_flatten_array)

    prediction_array = np.array(prediction)
    print(prediction_array.shape)

    # For each class
    n_classes = 3
    precision = dict()
    recall = dict()
    average_precision = dict()
    for i in range(n_classes):
        precision[i], recall[i], _ = precision_recall_curve(ground_truth_bin[:, i], prediction_array[:, i])
        average_precision[i] = average_precision_score(ground_truth_bin[:, i], prediction_array[:, i])
    plot_precision_recall(precision, recall, average_precision)

    # # Compute the performance metrics
    # model = new_clustering()
    # with open('prediction_data.sav', 'rb') as f:
    #     total_testing_data = pickle.load(f, encoding='latin1') 
    # model.verify(total_testing_data)

    # # Display the point cloud
    # display().publisher_frames(total_testing_data, single_frame = False)

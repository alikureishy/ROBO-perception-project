#!/usr/bin/env python

# Import modules
import tf
import rospy
import numpy as np
import sklearn
import time
import argparse
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.features import get_features
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

def downsample(pc, leaf_ratio=0.005):
    start = time.time()
    
    vox = pc.make_voxel_grid_filter()
    vox.set_leaf_size(*([leaf_ratio]*3))
    downsampled = vox.filter()

    end = time.time()
    latency = end - start
    print ("\tDownsampling: {} seconds".format(latency))
    return downsampled, latency
    
def clean(pc, mean_k=50, std_dev_mul_thresh=1.0):
    start = time.time()
    
    outlier_filter = pc.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(std_dev_mul_thresh) # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    cleaned = outlier_filter.filter()

    end = time.time()
    latency = end - start
    print ("\tCleaning: {} seconds".format(latency))
    return cleaned, latency

def slice(pc, field_name='z', limits=[0.75,1.1]):
    start = time.time()

    passthrough = pc.make_passthrough_filter()
    passthrough.set_filter_field_name(field_name)
    passthrough.set_filter_limits(*limits)
    sliced = passthrough.filter()

    end = time.time()
    latency = end - start
    print ("\tPassthrough: {} seconds".format(latency))    
    return sliced, latency

def segmentize(pc, distance_thresh=0.02):
    start = time.time()
    
    seg = pc.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(distance_thresh)
    points, coefficients = seg.segment()

    end = time.time()
    latency = end - start
    print ("\tRansac: {} seconds".format(latency))
    return points, latency

def separate_segments(pc, points):
    start = time.time()

    inliers_cloud = pc.extract(points, negative=False)
    outliers_cloud = pc.extract(points, negative=True)
    extraction_time = time.time()
    
    end = time.time()
    latency = end - start
    print ("\tExtraction: {} seconds".format(latency))
    return inliers_cloud, outliers_cloud, latency

def clusterize_objects(cloud):
    start = time.time()
    
    white_cloud = XYZRGB_to_XYZ(cloud)
    kdtree = white_cloud.make_kdtree()
    extractor = white_cloud.make_EuclideanClusterExtraction()
    extractor.set_ClusterTolerance(0.05) #0.1
    extractor.set_MinClusterSize(200)
    extractor.set_MaxClusterSize(4000) #8000
    extractor.set_SearchMethod(kdtree)
    clusters = extractor.Extract()  # array of index-clusters
    
    # Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(clusters))
    color_cluster_point_list = []
    for j, indices in enumerate(clusters):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([
                                                white_cloud[indice][0],
                                                white_cloud[indice][1],
                                                white_cloud[indice][2],
                                                rgb_to_float(cluster_color[j])
                                            ])
    clusterized = pcl.PointCloud_PointXYZRGB()
    clusterized.from_list(color_cluster_point_list)

    end = time.time()
    latency = end - start
    print ("\tClusterizing: {} seconds".format(latency))
    return clusterized, clusters, latency

def classify_objects(clusters, cloud, classifier, encoder, scaler):
    start = time.time()
    
    # Classify the clusters! (loop through each detected cluster one at a time)
    white_cloud = XYZRGB_to_XYZ(cloud)
    object_markers = []
    detected_objects = []
    for index, pts_list in enumerate(clusters):
        # Grab the points for the cluster
        pcl_cluster = cloud.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        features = get_features(ros_cluster)

        # Make the prediction
        prediction = classifier.predict(scaler.transform(features.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]

        # For publishing a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers.append(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    end = time.time()
    latency = end - start
    print ("\tClassification: {} seconds".format(latency))
    return detected_objects, object_markers, latency


"""
    downsampled, latency = downsample(pcl_raw, leaf_ratio=0.05)
    print ("\tDownsampling: {} seconds".format(latency))

    cleaned, latency = clean(downsampled, mean_k=50, std_dev_mul_thresh=1.0)
    print ("\tCleaning: {} seconds".format(latency))

    sliced, latency = slice(cleaned, field_name='z', limits=[0.75,1.1])
    print ("\tPassthrough: {} seconds".format(latency))

    inliers, latency = segmentize(sliced, distance_thresh=0.02)
    print ("\tRansac: {} seconds".format(latency))

    cloud_table, cloud_objects, latency = separate_segments(sliced, inliers)
    print ("\tExtraction: {} seconds".format(latency))

    cluster_cloud, latency = clusterize_objects(cloud_objects)
    print ("\Clusterizing: {} seconds".format(latency))

"""

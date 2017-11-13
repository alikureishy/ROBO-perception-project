#!/usr/bin/env python

# Import modules
import tf
import rospy
import numpy as np
import sklearn
from time import time
import argparse
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.features import get_features
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *
from sensor_stick.plotter import *

def downsample(pc, leaf_ratio=0.005):
    start = time()
    
    vox = pc.make_voxel_grid_filter()
    vox.set_leaf_size(*([leaf_ratio]*3))
    downsampled = vox.filter()

    end = time()
    latency = end - start
    print ("\tDownsampling: {} seconds".format(latency))
    return downsampled, latency
    
def clean(pc, mean_k=50, std_dev_mul_thresh=1.0):
    start = time()
    
    outlier_filter = pc.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(std_dev_mul_thresh) # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    cleaned = outlier_filter.filter()

    end = time()
    latency = end - start
    print ("\tCleaning: {} seconds".format(latency))
    return cleaned, latency

def slice(pc, field_name='z', limits=[0.75,1.1]):
    start = time()

    passthrough = pc.make_passthrough_filter()
    passthrough.set_filter_field_name(field_name)
    passthrough.set_filter_limits(*limits)
    sliced = passthrough.filter()

    end = time()
    latency = end - start
    print ("\tPassthrough: {} seconds".format(latency))    
    return sliced, latency

def segmentize(pc, distance_thresh=0.02):
    start = time()
    
    seg = pc.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(distance_thresh)
    points, coefficients = seg.segment()

    end = time()
    latency = end - start
    print ("\tRansac: {} seconds".format(latency))
    return points, latency

def separate_segments(pc, points):
    start = time()

    inliers_cloud = pc.extract(points, negative=False)
    outliers_cloud = pc.extract(points, negative=True)
    extraction_time = time()
    
    end = time()
    latency = end - start
    print ("\tExtraction: {} seconds".format(latency))
    return inliers_cloud, outliers_cloud, latency

def clusterize_objects(cloud, cluster_tolerance=0.05, min_size=200, max_size=4000, debug=False):
    start = time()
    
    white_cloud = XYZRGB_to_XYZ(cloud)
    kdtree = white_cloud.make_kdtree()
    extractor = white_cloud.make_EuclideanClusterExtraction()
    extractor.set_ClusterTolerance(cluster_tolerance) #0.1
    extractor.set_MinClusterSize(min_size)
    extractor.set_MaxClusterSize(max_size) #8000
    extractor.set_SearchMethod(kdtree)
    clusters = extractor.Extract()  # array of index-clusters
    
    # Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(clusters))
    color_cluster_point_list = []
    individual_object_clouds = []
    for j, indices in enumerate(clusters):
        object_points = []
        for i, indice in enumerate(indices):
            point = [
                        white_cloud[indice][0],
                        white_cloud[indice][1],
                        white_cloud[indice][2],
                        rgb_to_float(cluster_color[j])
                     ]
            object_points.append(point)
            color_cluster_point_list.append(point)
        if debug:
            individual_object = pcl.PointCloud_PointXYZRGB()
            individual_object.from_list(object_points)
            individual_object_clouds.append(individual_object)
    clusterized = pcl.PointCloud_PointXYZRGB()
    clusterized.from_list(color_cluster_point_list)

    end = time()
    latency = end - start
    print ("\tClusterizing: {} seconds".format(latency))
    return clusterized, clusters, individual_object_clouds, latency

def classify_objects(clusters, cloud, classifier, encoder, scaler, frame=None):
    start = time()
    
    # Classify the clusters! (loop through each detected cluster one at a time)
    white_cloud = XYZRGB_to_XYZ(cloud)
    object_markers = []
    detected_objects = []
    individual_object_clouds = []
    for index, pts_list in enumerate(clusters):
        # Grab the points for the cluster
        object_cloud = cloud.extract(pts_list)
        object_cloud_ros = pcl_to_ros(object_cloud)

        features = get_features(object_cloud_ros)

        # Make the prediction
        prediction = classifier.predict(scaler.transform(features.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        if frame is not None:
            graph = Graph("Prediction: {}".format(label), list(range(features.size)), features, "Bins", "Normalized Occurrences")
            frame.add(graph)
            
        # For publishing a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        marker = make_label(label,label_pos, index)
        object_markers.append(marker)

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = object_cloud_ros
        detected_objects.append(do)
        individual_object_clouds.append(object_cloud)

    end = time()
    latency = end - start
    print ("\tClassification: {} seconds".format(latency))
    return detected_objects, object_markers, individual_object_clouds, latency


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

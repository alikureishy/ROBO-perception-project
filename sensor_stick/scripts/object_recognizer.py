#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from sensor_stick.srv import GetNormals
from sensor_stick.features import get_features
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:
    # TODO: Convert ROS msg to PCL data
    pcl_raw = ros_to_pcl(pcl_msg)

    # TODO: Statistical noise removal
    outlier_filter = pcl_raw.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    outlier_filter.set_std_dev_mul_thresh(1.0) # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    cleaned = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    vox = cleaned.make_voxel_grid_filter()
    vox.set_leaf_size(*([0.008]*3))
    downsampled = vox.filter()

    # TODO: PassThrough Filter
    passthrough = downsampled.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
    passthrough.set_filter_limits(0.75, 1.1)
    sliced = passthrough.filter()
    
    # TODO: RANSAC Plane Segmentation
    seg = sliced.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.02)
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_table = sliced.extract(inliers, negative=False)
    cloud_objects = sliced.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    kdtree = white_cloud.make_kdtree()
    extractor = white_cloud.make_EuclideanClusterExtraction()
    extractor.set_ClusterTolerance(0.05) #0.1
    extractor.set_MinClusterSize(200)
    extractor.set_MaxClusterSize(4000) #8000
    extractor.set_SearchMethod(kdtree)
    cluster_indices = extractor.Extract()
    
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([
                                                white_cloud[indice][0],
                                                white_cloud[indice][1],
                                                white_cloud[indice][2],
                                                rgb_to_float(cluster_color[j])
                                            ])
                                            
    # Create new point cloud containing all clustered points, each assigned the appropriate color:
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_raw = pcl_to_ros(pcl_raw)
    ros_cleaned = pcl_to_ros(cleaned)
    ros_downsampled = pcl_to_ros(downsampled)
    ros_sliced = pcl_to_ros(sliced)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)

    # TODO: Publish ROS messages
    pcl_original_pub.publish(pcl_msg)
    pcl_raw_pub.publish(ros_raw)
    pcl_cleaned_pub.publish(ros_cleaned)
    pcl_downsampled_pub.publish(ros_downsampled)
    pcl_sliced_pub.publish(ros_sliced)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_cloud_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []
    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
    
        # Compute the associated feature vector
        #########################################
#        color_hist = compute_color_histograms(ros_cluster)
#        normals = get_normals(ros_cluster)
#        normal_hist = compute_normal_histograms(normals)
#        features = np.concatenate((color_hist, normal_hist)).astype(np.float64)
#        if index==0:
#            pcl_sample_pub.publish(ros_cluster)
        features = get_features(ros_cluster)
        #########################################
        
        # Make the prediction
        prediction = clf.predict(scaler.transform(features.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node("object_recognizer")

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_original_pub = rospy.Publisher("/pcl_original", PointCloud2, queue_size=1)
    pcl_raw_pub = rospy.Publisher("/pcl_raw", PointCloud2, queue_size=1)
    pcl_cleaned_pub = rospy.Publisher("/pcl_cleaned", PointCloud2, queue_size=1)
    pcl_downsampled_pub = rospy.Publisher("/pcl_downsampled", PointCloud2, queue_size=1)
    pcl_sliced_pub = rospy.Publisher("/pcl_sliced", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_cloud_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
#    pcl_sample_pub = rospy.Publisher("/pcl_sample", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

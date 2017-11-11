#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
import argparse
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import get_features
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import PickPlace
from rospy_message_converter import message_converter
import yaml

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    ###################
    # Exercise-2 TODOs:
    ###################
    # TODO: Convert ROS msg to PCL data
    pcl_raw = ros_to_pcl(pcl_msg)
    
    # TODO: Statistical Outlier Filtering
    outlier_filter = pcl_raw.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    outlier_filter.set_std_dev_mul_thresh(1.0) # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    cleaned = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    vox = cleaned.make_voxel_grid_filter()
    vox.set_leaf_size(*([0.005]*3))
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
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

#    # TODO: Convert PCL data to ROS messages
    ros_raw = pcl_to_ros(pcl_raw)
    ros_cleaned = pcl_to_ros(cleaned)
    ros_downsampled = pcl_to_ros(downsampled)
    ros_sliced = pcl_to_ros(sliced)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)

#    # TODO: Publish ROS messages
    pcl_original_pub.publish(pcl_msg)
    pcl_raw_pub.publish(ros_raw)
    pcl_cleaned_pub.publish(ros_cleaned)
    pcl_downsampled_pub.publish(ros_downsampled)
    pcl_sliced_pub.publish(ros_sliced)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    ###################
    # Exercise-3 TODOs:
    ###################

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

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
#    try:
#        pr2_mover(detected_objects)
#    except rospy.ROSInterruptException:
#        pass

class PickPlaceRequest(object):
    def __init__(self):
        self.test_scene_num = Int32()
        self.object_name = String()
        self.arm_name = String()
        self.pick_pose = Pose()
        self.pick_pose.position.x = 0
        self.pick_pose.position.y = 0
        self.pick_pose.position.z = 0
        self.pick_pose.orientation.x = 0
        self.pick_pose.orientation.y = 0
        self.pick_pose.orientation.z = 0
        self.pick_pose.orientation.w = 0
        self.place_pose = Pose()
        self.place_pose.position.x = 0
        self.place_pose.position.y = 0
        self.place_pose.position.z = 0
        self.place_pose.orientation.x = 0
        self.place_pose.orientation.y = 0
        self.place_pose.orientation.z = 0
        self.place_pose.orientation.w = 0

def do_pick_and_place(pick_place_request):
    # Wait for 'pick_place_routine' service to come up:
    rospy.wait_for_service('pick_place_routine')

    try:
        pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
        # TODO: Insert your message variables to be sent as a service request:
        resp = pick_place_routine(pick_place_request.test_scene_num, \
                                  pick_place_request.object_name, \
                                  pick_place_request.arm_name, \
                                  pick_place_request.pick_pose, \
                                  pick_place_request.place_pose)
        print ("Response: ",resp.success)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# Rotate PR2 in place to capture side tables for the collision map
def do_capture_sides():
    pass

# function to load parameters and request PickPlace service
def pr2_mover(detected_objects):
    # Map: Group -> Dropboxes #
    box_list = rospy.get_param('/dropbox')
    group_dict = {}
    for box in box_list:
        group_dict[box.group] = [box.name, box.position]

    # Map: Model -> Centroids #
    centroid_dict = {} # to be lookup of model --> list of tuples (x, y, z)
    for obj in detected_objects:
        points_arr = ros_to_pcl(obj.cloud).to_array()
        centroid = np.asscalar(np.mean(points_arr, axis=0)[:3])
        centroids[obj.label] = centroids.get(obj.label, []).append(centroid)

    # Rotate PR2 to capture side-tables for the collision map:
    do_capture_sides()

    # List: Yaml detections #
    object_list = rospy.get_param('/object_list')
    yaml_list = []
    for i, entry in enumerate(object_list):
        model = entry['name']
        group = entry['group']
        
        centroids = centroid_dict [model]
        if centroids is not None:
            for centroid in centroids:
                # Get group/box info:
                box_name, box_position = group_dict[group]
            
                # Create PickPlace request
                pick_place_request = PickPlaceRequest()
                pick_place_request.test_scene_num.data = args.test_scene
                pick_place_request.object_name.data = model
                pick_place_request.arm_name.data = box_name # arm_name = box_name
                pick_place_request.pick_pose.position.x = centroid[0]
                pick_place_request.pick_pose.position.y = centroid[1]
                pick_place_request.pick_pose.position.z = centroid[2]
                pick_place_request.place_pose.position.x = box_position[0]
                pick_place_request.place_pose.position.y = box_position[1]
                pick_place_request.place_pose.position.z = box_position[2]

                yaml_list.append(pick_place_request)
#                do_pick_and_place(pick_place_request)

    # Save all collected pick-place requests into a yaml file:
    send_to_yaml(args.outfile, yaml_list)

if __name__ == '__main__':
    # TODO: ROS node initialization
    rospy.init_node("advanced_pick_and_place")
    parser = argparse.ArgumentParser(description='Perform advanced pick+place')
    parser.add_argument('-i', dest="infile", required=True, type=str, help='Model file for the object recognition')
    parser.add_argument('-t', dest="test_scene", required=True, type=int, help='Test scene number')
    parser.add_argument('-o', dest='outfile', required=True, help='YAML file to save the generated pick+place request sequence')

    args = parser.parse_args()

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_original_pub = rospy.Publisher("/pcl_original", PointCloud2, queue_size=1)
    pcl_raw_pub = rospy.Publisher("/pcl_raw", PointCloud2, queue_size=1)
    pcl_cleaned_pub = rospy.Publisher("/pcl_cleaned", PointCloud2, queue_size=1)
    pcl_downsampled_pub = rospy.Publisher("/pcl_downsampled", PointCloud2, queue_size=1)
    pcl_sliced_pub = rospy.Publisher("/pcl_sliced", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open(args.infile, 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()    

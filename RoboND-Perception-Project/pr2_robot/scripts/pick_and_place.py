#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from time import time
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
from sensor_stick.pipeline import *

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
#    print("List length: {}".format(len(dict_list)))
#    print("\tList: {}".format(dict_list))
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    print ("Received capture:")
    start = time()

    ###################
    # Exercise-2 TODOs:
    ###################
    image = ros_to_pcl(pcl_msg)
    print ("\tDeserialization: {} seconds".format(time() - start))
    
    image, _ = downsampled, latency = downsample(image, leaf_ratio=0.003)

    image, _ = cleaned, latency = clean(image, mean_k=50, std_dev_mul_thresh=1.0)

    image, _ = sliced1, latency = slice(image, field_name='z', limits=[0.6,1.5])
    image, _ = sliced2, latency = slice(image, field_name='y', limits=[-0.4,0.4])

    inliers, latency = segmentize(image, distance_thresh=0.025)
    table_cloud, non_table_cloud, latency = separate_segments(image, inliers)

    objects_cloud, clusters, _, latency = clusterize_objects(non_table_cloud, cluster_tolerance=0.04, min_size=200, max_size=5000, debug=False)

#    # Convert PCL data to ROS messages
#    ros_raw = pcl_to_ros(pcl_raw)
#    ros_downsampled = pcl_to_ros(downsampled)
#    ros_cleaned = pcl_to_ros(cleaned)
#    ros_sliced = pcl_to_ros(sliced)
    ros_objects_cloud = pcl_to_ros(objects_cloud)
    ros_cloud_table = pcl_to_ros(table_cloud)
    ros_cloud_objects = pcl_to_ros(non_table_cloud)
    serialization_time = time()
#    print ("\tSerialization: {} seconds".format(serialization_time - masking_time))

#    # Publish ROS messages
#    pcl_original_pub.publish(pcl_msg)
#    pcl_raw_pub.publish(ros_raw)
#    pcl_downsampled_pub.publish(ros_downsampled)
#    pcl_cleaned_pub.publish(ros_cleaned)
#    pcl_sliced_pub.publish(ros_sliced)
    pcl_cluster_pub.publish(ros_objects_cloud)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_objects_pub.publish(ros_cloud_objects)
    publishing_time = time()
#    print ("\tPublishing: {} seconds. Clusters found: {}".format(publishing_time - serialization_time, len(cluster_indices)))

    ###################
    # Exercise-3 TODOs:
    ###################
    detections, markers, object_clouds, latency = classify_objects(clusters, non_table_cloud, classifier, encoder, scaler)
    assert len(detections) == len(clusters) == len(markers), \
            "{} clusters were identified, but {} were classified, and {} were marked.".format(len(detections), len(clusters), len(markers))
    print ("\tFound {} objects: {}".format(len(detections), list(map(lambda x: x.label, detections))))

    # Publish the list of detected objects
    for marker in markers:
        object_markers_pub.publish(marker)
    detected_objects_pub.publish(detections)

    end = time()
    print ("\tTotal: {} seconds".format(end - start))

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detections)
    except rospy.ROSInterruptException:
        pass

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

import pdb
        
# function to load parameters and request PickPlace service
def pr2_mover(detected_objects):
    # Map: Group -> Dropboxes #
    box_list = rospy.get_param('/dropbox')
    assert(len(box_list) > 0)
    group_dict = {}
    for box in box_list:
        group_dict[box['group']] = [box['name'], box['position']]

    # Map: Model -> Centroids #
    centroid_dict = {} # to be lookup of model --> list of tuples (x, y, z)
    assert(len(detected_objects) > 0)
    print("Calculating centroids")
    for obj in detected_objects:
        points_arr = ros_to_pcl(obj.cloud).to_array()
        means = np.mean(points_arr, axis=0)
        centroid = means[:3] # np.asscalar(means[:3])
        centroid = map(lambda x: float(x), centroid)
        centroid_dict[obj.label] = centroid_dict.get(obj.label, [])
        centroid_dict[obj.label].append(centroid)
        print("\t{}: {}".format(obj.label, centroid))

    # Rotate PR2 to capture side-tables for the collision map:
    do_capture_sides()

    # List: Yaml detections #
    object_list = rospy.get_param('/object_list')
    assert (len(object_list) > 0)
    yaml_list = []
    for i, entry in enumerate(object_list):
        model = entry['name']
        group = entry['group']
        print("{} -> {}:".format(model, group))
        
        centroids = centroid_dict.get(model, None)
        print("\t{} -> {} centroids".format(model, len(centroids) if centroids is not None else None))
        if centroids is not None:
            for centroid in centroids:
                print("\t\tCentroid: {}".format(centroid))
                # Get group/box info:
                box_name, box_position = group_dict[group]
                print("\t\t\t{} -> {}, {}".format(group, box_name, box_position))
            
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

                yaml_dict = make_yaml_dict(pick_place_request.test_scene_num, \
                                            pick_place_request.arm_name, \
                                            pick_place_request.object_name, \
                                            pick_place_request.pick_pose, \
                                            pick_place_request.place_pose)
                yaml_list.append(yaml_dict)
#                do_pick_and_place(pick_place_request)

    # Save all collected pick-place requests into a yaml file:
#    pdb.set_trace()
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
    classifier = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()    

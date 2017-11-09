#!/usr/bin/env python
import numpy as np
import rospy
import argparse
import yaml
import pickle

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

if __name__ == '__main__':
    rospy.init_node('capture_node')
    
    parser = argparse.ArgumentParser(description='Capture point clouds for feature extraction')
    parser.add_argument('-y', dest="yaml", required=True, type=str, help='YAML file with model names')
    parser.add_argument('-i', dest='iterations', default=10, type=int, help='Num of variations (default: 10)')
    parser.add_argument('-o', dest='outfile', default = "point_clouds.pickle", help='Pickle file to save point clouds')
    parser.add_argument('-t', dest='topic', default = None, help='Topic to which to publish the sampled point clouds')

    args = parser.parse_args()
    pcl_pub = None
    if args.topic is not None:
        pcl_pub = rospy.Publisher(args.topic, PointCloud2, queue_size=1)
    models = []
    print ("Reading object types from: {}".format(args.yaml))
    data = yaml.load(open(args.yaml))
    object_list = data['object_list']
    print ("YAML contained {} objects:\n{}\n".format(len(object_list), object_list))
    for i, model in enumerate(object_list):
        models.append(model['name'])
    print ("Models found: ", models)

#    pcl_sample_pub = rospy.Publisher("/pcl_sample", PointCloud2, queue_size=1)
#    models = [\
#       'beer',
#       'bowl',
#       'create',
#       'disk_part',
#       'hammer',
#       'plastic_cup',
#       'soda_can']

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_point_clouds = []

    for model_name in models:
        print ("Generating {} samples for model: {}".format(args.iterations, model_name))
        spawn_model(model_name)

        for i in range(args.iterations):
            print (model_name, i)
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                print(type(sample_cloud))
                sample_cloud_pcl = ros_to_pcl(sample_cloud)
                print(type(sample_cloud_pcl))
                sample_cloud_arr = sample_cloud_pcl.to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            if sample_was_good:
                labeled_point_clouds.append({'name': model_name, 'pointcloud': sample_cloud_pcl})
                if pcl_pub is not None:
                    pcl_pub.publish(sample_cloud)
        delete_model()

    print ("Storing labeled point clouds in file: {}".format(args.outfile))
    pickle.dump(labeled_point_clouds, open(args.outfile, 'wb'))
    print ("Point cloud sampling comlete!")


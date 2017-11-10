#!/usr/bin/env python
import numpy as np
import rospy
import argparse
import yaml
import pickle
import os
from os import makedirs
from os.path import isfile, isdir

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

class FileNamer(object):
    def __init__(self, folder, prefix, extension, maxfiles=1000, overwrite=False):
        self.__folder__ = folder
        self.__prefix__ = prefix
        self.__extension__ = extension
        self.__max_files__ = maxfiles

    def get_for_idx(self, idx):
        return os.path.join(self.__folder__, '{}_{}.{}'.format(self.__prefix__, idx, self.__extension__))
    
    def get_file_count(self):
        count = 0
        for idx in range(1, self.__max_files__+1):
            if not isfile(self.get_for_idx(idx)):
                count = idx-1
                break
        return count
    
    def next_vacant_name(self):
        # If no folder exists, no point looking
        if not isdir(self.__folder__):
            makedirs(self.__folder__)

        # Determine next index. Throw error after __max_files__ reached
        count = self.get_file_count()
        if count >= self.__max_files__:
            raise "File count in {} has reached capacity of {}".format(self.__folder__, self.__max_files__)
        else:
            filename = self.get_for_idx(count+1)
        return filename
        
if __name__ == '__main__':
    rospy.init_node('capture_node')
    
    parser = argparse.ArgumentParser(description='Capture point clouds for feature extraction')
    parser.add_argument('-y', dest="yaml", required=True, type=str, help='YAML file with model names')
    parser.add_argument('-c', dest='count', default=10, type=int, help='Num of variations (default: 10)')
    parser.add_argument('-o', dest='outfolder', default = os.getcwd(), help='Folder to save .pcd files for point cloud samples')
    parser.add_argument('-t', dest='topic', default = None, help='Topic to which to publish the sampled point clouds')

    args = parser.parse_args()
    pcl_pub = None
    if args.topic is not None:
        pcl_pub = rospy.Publisher(args.topic, PointCloud2, queue_size=1)
    print ("Reading object types from: {}".format(args.yaml))
    data = yaml.load(open(args.yaml))
    object_list = data['object_list']
    print ("YAML contained {} objects:\n{}\n".format(len(object_list), object_list))
    filenamers = {}
    models = []
    for i, entry in enumerate(object_list):
        model_name = entry['name']
        models.append(model_name)
        filenamers[model_name] = FileNamer(os.path.join(args.outfolder, model_name), "pc_{}".format(model_name), "pcd")
    print ("Models found: ", models)

    if args.topic is not None:
        pcl_sample_pub = rospy.Publisher("/pcl_sample", PointCloud2, queue_size=1)

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_point_clouds = []

    for model_name in models:
        print ("Generating {} samples for model: {}".format(args.count, model_name))
        spawn_model(model_name)
        filenamer = filenamers[model_name]

        for i in range(args.count):
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_pcl = ros_to_pcl(sample_cloud)
                sample_cloud_arr = sample_cloud_pcl.to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('\t\tInvalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            if sample_was_good:
                pclfile = filenamer.next_vacant_name()
                print("\t{} -> {}".format(i, pclfile))
                pcl.save(sample_cloud_pcl, pclfile, format="pcd")
                if pcl_pub is not None:
                    pcl_pub.publish(sample_cloud)
        delete_model()

    print ("Point cloud sampling comlete!")


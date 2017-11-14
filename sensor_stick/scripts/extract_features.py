#!/usr/bin/env python
import numpy as np
import os
import pickle
import yaml
import rospy
import random
import argparse
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import time

from sensor_stick.pcl_helper import *
from sensor_stick.plotter import *
from sensor_stick.features import get_features
from sensor_msgs.msg import PointCloud2
from capture_point_clouds import FileNamer
from sensor_stick.pipeline import downsample

def pickleloader(filename):
    with open(filename, "rb") as f:
        while True:
            try:
                yield pickle.load(f)
            except EOFError:
                break

def preprocess(point_cloud):
    point_cloud, _ = downsample(point_cloud, leaf_ratio=0.003)
    return point_cloud

if __name__ == '__main__':
    rospy.init_node("extraction_node")
    parser = argparse.ArgumentParser(description='Capture point clouds for feature extraction')
    parser.add_argument('-i', dest="infolder", required=True, type=str, help='Root folder containing model-based sub-folders of point cloud samples')
    parser.add_argument('-y', dest="yaml", required=True, type=str, help='YAML file with model names')
    parser.add_argument('-c', dest='count', required=True, type=int, help='Num of variations of each model to process (default: 10)')
    parser.add_argument('-o', dest='outfile', required=True, help='Pickle file to save extracted features')
    parser.add_argument('-p', dest='plot', action="store_true", default = False, help='Whether to plot the feature histograms (default: False)')

    args = parser.parse_args()
    illustrator = Illustrator(True)
    
    print ("Reading object types from: {}".format(args.yaml))
    data = yaml.load(open(args.yaml))
    object_list = data['object_list']
    print ("YAML contained {} models:\n{}\n".format(len(object_list), object_list))
    filenamers = {}
    samplecounts = {}
    total_available_samples = 0
    print ("Calculating sample availability:")
    for entry in object_list:
        model = entry['name']
        filenamers[model] = FileNamer(os.path.join(args.infolder, model), "pc_{}".format(model), "pcd", maxfiles=args.count)
        available_samples = filenamers[model].get_file_count()
        samplecounts[model] = available_samples
        total_available_samples += available_samples
        print("\t{}: {}".format(model, available_samples))
        if available_samples < args.count:
            print ("\t\tWARNING: {} does not have sufficient samples available. Only {} available.".format(model, available_samples))
    print ("Total available sample count: {}".format(total_available_samples))

    labeled_features = []
    for i, entry in enumerate(object_list):
        model_name = entry['name']
        filenamer = filenamers[model_name]
        print("{}: Model: {}. Avaiable samples {}. Selecting {}...".format(i, model_name, samplecounts[model_name], args.count))
        for j in range(0, args.count):
            r = random.randint(1, samplecounts[model_name])
            pcfile = filenamer.get_for_idx(r)
            print("\t{}: Processing {} ...".format(j, pcfile))
            point_cloud = pcl.load_XYZRGB(pcfile)
            
            # Any pre-processing before feature extraction:
            point_cloud = preprocess(point_cloud)
            
            point_cloud_ros = pcl_to_ros(point_cloud)
            features = get_features(point_cloud_ros)
            labeled_features.append([features, model_name])
        
            # Plot
            if args.plot:
                frame = illustrator.nextframe((i*args.count) + j)
                frame.newsection(model_name)
                graph = Graph(model_name, list(range(features.size)), features, "Bins", "Normalized Occurrences")
                frame.add(graph)
                frame.render()

    print ("Storing features in file: {}".format(args.outfile))
    pickle.dump(labeled_features, open(args.outfile, 'wb'))
    print("Feature extraction complete!")


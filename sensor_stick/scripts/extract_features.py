#!/usr/bin/env python
import numpy as np
import pickle
import rospy
import argparse
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import time

from sensor_stick.pcl_helper import *
from sensor_stick.plotter import *
from sensor_stick.features import get_features
from sensor_msgs.msg import PointCloud2

def pickleloader(filename):
    with open(filename, "rb") as f:
        while True:
            try:
                yield pickle.load(f)
            except EOFError:
                break

if __name__ == '__main__':
    rospy.init_node("extraction_node")
    parser = argparse.ArgumentParser(description='Capture point clouds for feature extraction')
    parser.add_argument('-i', dest="infile", required=True, type=str, help='Pickle file with labeled point cloud samples')
    parser.add_argument('-o', dest='outfile', default = "features.pickle", help='Pickle file to save extracted features')
    parser.add_argument('-p', dest='plot', action="store_true", default = False, help='Whether to plot the feature histograms (default: False)')

    args = parser.parse_args()
    illustrator = Illustrator(True)
    print ("Reading labeled point cloud samples from: {}".format(args.infile))
    labeled_point_clouds = pickle.load(open(args.infile))
    print ("Sample count: {}".format(len(labeled_point_clouds)))

    labeled_features = []

    for i, entry in enumerate(labeled_point_clouds):
        model_name = entry['name']
        frame = illustrator.nextframe(i)
        print("{}: Model: {}. Extracting sample features.".format(i, model_name))
        point_cloud = pcl_to_ros(entry['pointcloud'])
        features = get_features(point_cloud)
        labeled_features.append([features, model_name])
        
        # Plot
        if args.plot:
            frame.newsection(model_name)
            graph = Graph(model_name, range(features.size), features, "Bins", "Normalized Occurrences")
            frame.add(graph)
            frame.render()

    print ("Storing features in file: {}".format(args.outfile))
    pickle.dump(labeled_features, open(args.outfile, 'wb'))
    print("Feature extraction complete!")


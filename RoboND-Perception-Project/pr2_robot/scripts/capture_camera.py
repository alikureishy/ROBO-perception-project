#!/usr/bin/env python

# Import modules
import tf
import rospy
import numpy as np
import argparse
import os
import pickle
from os.path import isfile, isdir
from os import makedirs
from sensor_stick.pcl_helper import *
from sensor_stick.pipeline import *
from sensor_stick.plotter import *
from time import time
import sys

from argparse import ArgumentParser, ArgumentTypeError
import re

def parseNumList(string):
    m = re.match(r'(\d+)(?:-(\d+))?$', string)
    # ^ (or use .split('-'). anyway you like.)
    if not m:
        raise ArgumentTypeError("'" + string + "' is not a range of number. Expected forms like '0-5' or '2'.")
    start = m.group(1)
    end = m.group(2) or start
    return list(range(int(start,10), int(end,10)+1))

if __name__ == '__main__':
    # TODO: ROS node initialization
    parser = argparse.ArgumentParser(description='Perform advanced pick+place')
    parser.add_argument('-i', dest="infile", required=True, type=str, help='Model file for the object recognition')
    parser.add_argument('-t', dest="topic", required=True, type=str, help="Name of topic to capture. ['/pr2/world/points', ....]")
    parser.add_argument('-c', dest="count", default=0, type=int, help='Number of times to snapshot')
    parser.add_argument('-l', dest="levels", nargs='*', default=list(range(0,6)), type=int, help='List of stages whose outputs will be saved to disk [0 = Original] [1 = Downsampled] [2 = Cleaned] [3 = Sliced] [4 = Segmented] [5 = Object Cloud] [6 = Detected Objects]')
    parser.add_argument('-o', dest='outfolder', required=True, help="Folder where all the pipeline PCDs will be saved")
    parser.add_argument('-p', dest='plot', action="store_true", default = False, help='Whether to plot the feature histograms (default: False)')
    args = parser.parse_args()
    print (args)

    if args.plot:
        illustrator = Illustrator(True)
    counter = 1

    rospy.init_node("camera_snapshot")
    def pcl_callback(pcl_msg):
        global counter
        global illustrator
        print ("{}: Received capture from topic: {}".format(counter, args.topic))
        if (counter <= args.count):
            folder = os.path.join(args.outfolder, str(counter))
            if not isdir(folder):
                makedirs(folder)

            start_time = time()
            
            pcl_raw = image = ros_to_pcl(pcl_msg)
            print ("\tDeserialization: {} seconds".format(time() - start_time))
            
            # Original
            if 0 in args.levels:
                pcl.save(image, os.path.join(folder, "original.pcd"), format="pcd")
            
            # Downsampling
            image, _ = downsampled, latency = downsample(image, leaf_ratio=0.003)
            if 1 in args.levels:
                pcl.save(downsampled, os.path.join(folder, "downsampled.pcd"), format="pcd")

            # CLeaning
            image, _ = cleaned, latency = clean(image, mean_k=50, std_dev_mul_thresh=1.0)
            if 2 in args.levels:
                pcl.save(cleaned, os.path.join(folder, "cleaned.pcd"), format="pcd")

            # Slicing
            image, _ = sliced1, latency = slice(image, field_name='z', limits=[0.5,1.5])
            image, _ = sliced2, latency = slice(image, field_name='y', limits=[-0.4,0.4])
            if 3 in args.levels:
                pcl.save(sliced1, os.path.join(folder, "slice1.pcd"), format="pcd")
                pcl.save(sliced2, os.path.join(folder, "slice2.pcd"), format="pcd")

            # Segmentaion and clustering
            inliers, latency = segmentize(image, distance_thresh=0.025)
            table_cloud, non_table_cloud, latency = separate_segments(image, inliers)
            if 4 in args.levels:
                pcl.save(table_cloud, os.path.join(folder, "table.pcd"), format="pcd")
                pcl.save(non_table_cloud, os.path.join(folder, "non-table.pcd"), format="pcd")

            # Only if we're doing segmentation:
            objects_cloud, clusters, _, latency = clusterize_objects(non_table_cloud, debug=False)
            if 5 in args.levels:
                pcl.save(objects_cloud, os.path.join(folder, "objects.pcd"), format="pcd")

            frame = None
            if args.plot:
                frame = illustrator.nextframe(counter)
                frame.newsection(counter)
        
            model = pickle.load(open(args.infile, 'rb'))
            classifier = model['classifier']
            encoder = LabelEncoder()
            encoder.classes_ = model['classes']
            scaler = model['scaler']
            detections, markers, object_clouds, latency = classify_objects(clusters, non_table_cloud, classifier, encoder, scaler, frame)

            if 6 in args.levels:
                # Save the objects into individual files:
                for i, cloud in enumerate(object_clouds):
                    label = detections[i].label
                    pcl.save(cloud, os.path.join(folder, "{}_{}.pcd".format(i, label)), format="pcd")
                        
            print ("\tFound {} objects: {}".format(len(detections), list(map(lambda x: x.label, detections))))
        
            if args.plot:
                frame.render()
                
            counter += 1
        else:
            print ("\tSnapshots taken. Skipping.")
            sys.exit(0)

    pcl_sub = rospy.Subscriber(args.topic, PointCloud2, pcl_callback, queue_size=1)

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

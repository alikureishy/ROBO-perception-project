#!/usr/bin/env python

# Import modules
import tf
import rospy
import numpy as np
import time
import argparse
import os
from os.path import isfile, isdir
from os import makedirs
from sensor_stick.pcl_helper import *
from sensor_stick.pipeline import *

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
    parser.add_argument('-t', dest="topic", required=True, type=str, help="Name of topic to capture. ['/pr2/world/points', ....]")
    parser.add_argument('-c', dest="countdown", default=0, type=int, help='Number of callbacks to wait before capture')
    parser.add_argument('-l', dest="levels", nargs='*', default=list(range(0,6)), type=int, help='List of stages to perform [0 = Original] [1 = Downsampled] [2 = Cleaned] [3 = Sliced] [4 = Segmented [5 = Clustered]]')
    parser.add_argument('-o', dest='outfolder', required=True, help="Folder where all the pipeline PCDs will be saved")
    args = parser.parse_args()
    print (args)

    rospy.init_node("camera_snapshot")
    counter = args.countdown
    def pcl_callback(pcl_msg):
        global counter
        print ("Received capture from topic: {}".format(args.topic))
        if (counter >= 0):
            if not isdir(args.outfolder):
                makedirs(args.outfolder)
            start_time = time.time()
            
            pcl_raw = image = ros_to_pcl(pcl_msg)
            print ("\tDeserialization: {} seconds".format(time.time() - start_time))
            
            # Original
            if 0 in args.levels:
                pcl.save(image, os.path.join(args.outfolder, "original.pcd"), format="pcd")
            
            # Downsampling
            if 1 in args.levels:
                image, _ = downsampled, latency = downsample(image, leaf_ratio=0.05)
                pcl.save(downsampled, os.path.join(args.outfolder, "downsampled.pcd"), format="pcd")

            # CLeaning
            if 2 in args.levels:
                image, _ = cleaned, latency = clean(image, mean_k=50, std_dev_mul_thresh=1.0)
                pcl.save(cleaned, os.path.join(args.outfolder, "cleaned.pcd"), format="pcd")

            # Slicing
            if 3 in args.levels:
                image, _ = sliced, latency = slice(image, field_name='z', limits=[0.75,1.1])
                pcl.save(sliced, os.path.join(args.outfolder, "sliced.pcd"), format="pcd")

            # Segmentaion and clustering
            if 4 in args.levels:
                inliers, latency = segmentize(image, distance_thresh=0.02)
                table_cloud, non_table_cloud, latency = separate_segments(image, inliers)
                pcl.save(table_cloud, os.path.join(args.outfolder, "table.pcd"), format="pcd")
                pcl.save(non_table_cloud, os.path.join(args.outfolder, "non-table.pcd"), format="pcd")

                # Only if we're doing segmentation:
                if 5 in args.levels:
                    objects_cloud, latency = clusterize_objects(non_table_cloud)
                    pcl.save(objects_cloud, os.path.join(args.outfolder, "objects.pcd"), format="pcd")

            counter -= 1
        else:
            print ("\tSnapshot taken. Skipping.")

    pcl_sub = rospy.Subscriber(args.topic, PointCloud2, pcl_callback, queue_size=1)

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

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

if __name__ == '__main__':
    # TODO: ROS node initialization
    parser = argparse.ArgumentParser(description='Perform advanced pick+place')
    parser.add_argument('-t', dest="topic", required=True, type=str, help='Name of topic to capture. [/pr2/world/points, ....]')
    parser.add_argument('-c', dest="countdown", default=0, type=int, help='Number of callbacks to wait before capture')
    parser.add_argument('-o', dest='outfolder', required=True, help="Folder where all the pipeline PCDs will be saved")
    args = parser.parse_args()

    rospy.init_node("camera_snapshot")
    counter = args.countdown
    def pcl_callback(pcl_msg):
        global counter
        print ("Received capture from topic: {}".format(args.topic))
        if (counter >= 0):
            if not isdir(args.outfolder):
                makedirs(args.outfolder)
            start_time = time.time()
            pcl_raw = ros_to_pcl(pcl_msg)
            print ("\tDeserialization: {} seconds".format(time.time() - start_time))
            pcl.save(pcl_raw, os.path.join(args.outfolder, "original.pcd"), format="pcd")
            
            downsampled, latency = downsample(pcl_raw, leaf_ratio=0.05)
            pcl.save(downsampled, os.path.join(args.outfolder, "downsampled.pcd"), format="pcd")

            cleaned, latency = clean(downsampled, mean_k=50, std_dev_mul_thresh=1.0)
            pcl.save(cleaned, os.path.join(args.outfolder, "cleaned.pcd"), format="pcd")

            sliced, latency = slice(cleaned, field_name='z', limits=[0.75,1.1])
            pcl.save(sliced, os.path.join(args.outfolder, "sliced.pcd"), format="pcd")

            inliers, latency = segmentize(sliced, distance_thresh=0.02)

            table_cloud, non_table_cloud, latency = separate_segments(sliced, inliers)
            pcl.save(table_cloud, os.path.join(args.outfolder, "table.pcd"), format="pcd")
            pcl.save(non_table_cloud, os.path.join(args.outfolder, "non-table.pcd"), format="pcd")

            objects_cloud, latency = clusterize_objects(non_table_cloud)
            pcl.save(objects_cloud, os.path.join(args.outfolder, "objects.pcd"), format="pcd")

            counter -= 1
        else:
            print ("\tSnapshot taken. Skipping.")

    pcl_sub = rospy.Subscriber(args.topic, PointCloud2, pcl_callback, queue_size=1)

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

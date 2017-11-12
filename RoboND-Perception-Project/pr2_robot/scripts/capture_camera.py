#!/usr/bin/env python

# Import modules
import tf
import rospy
import numpy as np
import time
import argparse
from sensor_stick.pcl_helper import *

if __name__ == '__main__':
    # TODO: ROS node initialization
    parser = argparse.ArgumentParser(description='Perform advanced pick+place')
    parser.add_argument('-t', dest="topic", required=True, type=str, help='Name of topic to capture. [/pr2/world/points, ....]')
    parser.add_argument('-c', dest="countdown", default=0, type=int, help='Number of callbacks to wait before capture')
    parser.add_argument('-o', dest='outfile', required=True, help='PCD file to which to save the capture point cloud')
    args = parser.parse_args()

    rospy.init_node("camera_snapshot")
    counter = args.countdown
    def pcl_callback(pcl_msg):
        global counter
        print ("Received capture. Counter: {}".format(counter))
        if (counter >= 0):
            start_time = time.time()
            pcl_raw = ros_to_pcl(pcl_msg)
            deserialization_time = time.time()
            print ("\tDeserialization: {} seconds".format(deserialization_time - start_time))

            # Voxel Grid Downsampling
            vox = pcl_raw.make_voxel_grid_filter()
            vox.set_leaf_size(*([0.01]*3))
            downsampled = vox.filter()
            downsampling_time = time.time()
            print ("\tDownsampling: {} seconds".format(downsampling_time - deserialization_time))
            counter -= 1
            print("\tTaking snapshot: {} -> {}".format(args.topic, args.outfile))
            pcl.save(downsampled, args.outfile, format="pcd")
        else:
            print ("\tSnapshot taken. Skipping.")

    pcl_sub = rospy.Subscriber(args.topic, PointCloud2, pcl_callback, queue_size=1)

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

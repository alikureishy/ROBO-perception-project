#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    pcl_raw = ros_to_pcl(pcl_msg)
    
    # TODO: Convert (back) PCL object to ROS messages
    ros_raw = pcl_to_ros(pcl_raw)

    # TODO: Publish ROS messages
    pcl_original_pub.publish(pcl_msg)
    pcl_raw_pub.publish(ros_raw)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node("advanced_pick_and_place")

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_original_pub = rospy.Publisher("/pcl_original", PointCloud2, queue_size=1)
    pcl_raw_pub = rospy.Publisher("/pcl_raw", PointCloud2, queue_size=1)

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

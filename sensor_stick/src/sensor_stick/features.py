import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *

def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized

def compute_color_histograms(cloud, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # TODO: Compute histograms
    channel_1_hist = np.histogram(channel_1_vals, bins=32, range=(0,256))
    channel_2_hist = np.histogram(channel_2_vals, bins=32, range=(0,256))
    channel_3_hist = np.histogram(channel_3_vals, bins=32, range=(0,256))

    # TODO: Concatenate and normalize the histograms
    total_hist = np.concatenate((channel_1_hist[0], channel_2_hist[0], channel_3_hist[0])).astype(np.float64)
    normed_features = total_hist / np.sum(total_hist)

    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    return normed_features 


def compute_normal_histograms(normal_cloud):
    print ("x")
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
#        norm_x_vals.append((norm_component[0] + 1.0) * 128.0) # Original range is -1.0 - 1.0. This expands it to 0-256 range.
#        norm_y_vals.append((norm_component[1] + 1.0) * 128.0) # Original range is -1.0 - 1.0. This expands it to 0-256 range.
#        norm_z_vals.append((norm_component[2] + 1.0) * 128.0) # Original range is -1.0 - 1.0. This expands it to 0-256 range.
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

#    print ("Max: ", np.max(norm_x_vals), np.max(norm_z_vals), np.max(norm_z_vals))
#    print ("Min: ", np.min(norm_x_vals), np.max(norm_z_vals), np.max(norm_z_vals))
    
#    norm_x_vals = np.multiply(np.add(norm_x_vals, 1.0), 128.0) # Original range is -1.0 - 1.0. This expands it to 0-256 range.
#    norm_y_vals = np.multiply(np.add(norm_y_vals, 1.0), 128.0) # Original range is -1.0 - 1.0. This expands it to 0-256 range.
#    norm_z_vals = np.multiply(np.add(norm_z_vals, 1.0), 128.0) # Original range is -1.0 - 1.0. This expands it to 0-256 range.
#    assert np.min(norm_x_vals) >= 0.0 and np.max(norm_x_vals) <= 256
#    assert np.min(norm_y_vals) >= 0.0 and np.max(norm_y_vals) <= 256
#    assert np.min(norm_z_vals) >= 0.0 and np.max(norm_z_vals) <= 256

    # TODO: Compute histograms
#    norm_x_hist = np.histogram(norm_x_vals, bins=32, range=(-1.0,1.0))
#    norm_y_hist = np.histogram(norm_y_vals, bins=32, range=(-1.0,1.0))
#    norm_z_hist = np.histogram(norm_z_vals, bins=32, range=(-1.0,1.0))
    norm_x_hist = np.histogram(norm_x_vals, bins=32, range=(-1.0,1.0))
    norm_y_hist = np.histogram(norm_y_vals, bins=32, range=(-1.0,1.0))
    norm_z_hist = np.histogram(norm_z_vals, bins=32, range=(-1.0,1.0))


    # TODO: Concatenate and normalize the histograms
    total_hist = np.concatenate((norm_x_hist[0], norm_y_hist[0], norm_z_hist[0])).astype(np.float64)
    normed_features = total_hist / np.sum(total_hist)

    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    return normed_features
    
#######################################################################################
from sensor_stick.srv import GetNormals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

def get_features(cloud):
    # Extract histogram features
    chists = compute_color_histograms(cloud, using_hsv=False)
    hhists = compute_color_histograms(cloud, using_hsv=True)
    normals = get_normals(cloud)
    nhists = compute_normal_histograms(normals)
    feature = np.concatenate((chists, hhists, nhists))
    return feature
#######################################################################################



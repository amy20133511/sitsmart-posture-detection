#!/usr/bin/env python
from data import *

import rospy
import rosbag
import numpy as np
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header

# Depth image parameters (adjust based on your camera)
image_rows_full = 480  # Height of the depth image
image_cols = 848       # Width of the depth image
frame_rate = 30        # Frame rate of the depth camera

max_range = 80.0       # Maximum depth to capture (in meters)
min_range = 0.1        # Minimum depth to capture (in meters)

# Convert ros bags with depth images to npy
def bag2np(data_set_name, image_topic, npy_file_name):
    """
    Convert all .bag files in a specific folder to a single .npy file containing 2D depth images.
    """
    print('#' * 50)
    print('Dataset name: {}'.format(data_set_name))
    
    # Initialize an empty array to store all depth images
    depth_image_array = np.empty([0, image_rows_full, image_cols, 1], dtype=np.float32)
    
    # Find all bag files in the given directory
    bag_file_path = os.path.join(data_set_name)
    bag_files = os.listdir(bag_file_path)
    
    print("Found bag files: ", bag_files)
    
    # Initialize CvBridge for converting ROS Image messages
    bridge = CvBridge()
    
    # Loop through all bags in the given directory
    for file_name in bag_files:
        # Bag file path
        file_path = os.path.join(bag_file_path, file_name)
        
        # Open ROS bags 
        with rosbag.Bag(file_path, 'r') as bag:
            # Loop through all messages in the bag
            for topic, msg, t in bag.read_messages(topics=[image_topic]):
                # Match the depth image topic
                if topic == image_topic:
                    print('Processing depth image message {}...'.format(depth_image_array.shape[0]))
                    
                    # Convert depth image to numpy array using CvBridge
                    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                    
                    # Normalize depth image based on min and max range
                    depth_image = np.clip(depth_image, min_range, max_range)
                    
                    # Scale the depth image to [0, 1] range
                    depth_image = (depth_image - min_range) / (max_range - min_range)
                    
                    # Add a new axis to match the desired shape (H, W, 1)
                    depth_image = depth_image[..., np.newaxis]
                    
                    # Append the depth image to the array
                    depth_image_array = np.append(depth_image_array, depth_image[np.newaxis, ...], axis=0)
    
    # Save full resolution image array as npy
    np.save(npy_file_name, depth_image_array)
    print('Dataset saved: {}'.format(npy_file_name))


if __name__ == '__main__':
    # Set up directory and file paths
    home_dir = os.path.expanduser('~')
    
    # Directory where the bag files are located
    data_set_name = os.path.join(home_dir, 'Documents', 'bags')
    
    # Your depth image ROS topic, e.g., '/camera/depth/image_raw'
    image_topic = '/camera/depth/image_raw'
    
    # The location where your npy file will be saved
    npy_file_name = os.path.join(home_dir, 'Documents', "depth_data.npy")
    
    # Convert the .bag files to a .npy file
    bag2np(data_set_name, image_topic, npy_file_name)
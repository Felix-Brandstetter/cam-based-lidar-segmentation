"""
Script for extracting data from rosbag
Usage:
    1.Enter params
    2. source ros-workspace (source /opt/ros/noetic/setup.bash)
"""
#params: 
path_to_rosbag = "./data/rosbags/real_data_calib.bag"
image_topic = "/ace1_driver/image_raw"
point_cloud_topic = "/os_cloud_node/points"
outputdir = "./data/Extracted-Data"


import tkinter as tk
from tkinter import filedialog
import os
from xmlrpc.client import Boolean
import cv2
from PIL import ImageTk
import numpy as np
import os
import yaml
import subprocess
import pathlib
import utils

import rosbag
from cv_bridge import CvBridge
import ros_numpy  # apt install ros-noetic-ros-numpy
import sensor_msgs


def extract_images(path_to_rosbag,image_topic,outputdir):
    try:
        path = os.path.join(outputdir, "images")
        utils.create_empty_dir(path)
        count = 0
        bag = rosbag.Bag(path_to_rosbag, "r")
        bridge = CvBridge()
        with open(os.path.join(path, "timestamps.txt"),"w",) as timestamp_file:
            for topic, msg, t in bag.read_messages(topics=[image_topic]):
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                cv2.imwrite(
                    os.path.join(path,str(msg.header.stamp.to_sec()) +".png"),cv_img,)
                timestamp_file.write("{}\n".format(msg.header.stamp.to_sec()))
                count += 1

        bag.close()
        print("Images Extracted")

    except Exception as e:
        print("Error + %s" %e)


def extract_pointclouds(path_to_rosbag,point_cloud_topic,outputdir):
    #Extract Pointclouds
    try:
        path = os.path.join(outputdir, "point_clouds")
        utils.create_empty_dir(path)
        count = 0
        with open(os.path.join(path, "timestamps.txt"),"w",) as timestamp_file:
            for topic, msg, t in rosbag.Bag(path_to_rosbag).read_messages(topics=[point_cloud_topic]):
                msg.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2
                offset_sorted = {f.offset: f for f in msg.fields}
                msg.fields = [f for (_, f) in sorted(offset_sorted.items())]
                pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
                np.save(os.path.join(path,str(msg.header.stamp.to_sec())), pc_np)
                timestamp_file.write("{}\n".format(msg.header.stamp.to_sec()))
                count += 1
        print("Pointclouds Extracted")

    except Exception as e:
        print("Error + %s" %e)


if __name__ == "__main__":
    extract_images(path_to_rosbag,image_topic,outputdir)
    extract_pointclouds(path_to_rosbag,point_cloud_topic,outputdir)

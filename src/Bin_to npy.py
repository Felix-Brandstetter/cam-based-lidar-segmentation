
"""
Script for converting bin files to np.array
Usage:
    1.Enter params
"""

#params
inputdir = "./data/simulation_data/lidar_data/velodyne"
outputdir = "./data/simulation_data/lidar_data/velodyne_npy"


import numpy as np
import glob
import os


count = 0
files = sorted(glob.glob(os.path.join(inputdir,"*.bin")))
for file in files:
    with open(file,"rb") as file:
        bytes=file.read()
        points=np.frombuffer(bytes,dtype="float32")
        points=points.reshape((-1,4))
        points = np.delete(points,3,1)
        points[:,1]= points[:,1]*-1
        np.save(os.path.join(outputdir,"pointcloud%008i" % count), points)
        count +=1


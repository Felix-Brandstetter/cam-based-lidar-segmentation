import os
import shutil

images = "./data/Extracted-Data/images"
pointclouds = "./data/Extracted-Data/point_clouds"
outputdir = "./data/Matched_timestamps"

for pointcloud in os.listdir(pointclouds):
    for image in os.listdir(images):
        time_pointcloud = float(pointcloud.replace(".npy",""))
        time_image = float(image.replace(".png",""))
        if abs(time_pointcloud - time_image )<0.001:
            shutil.copy(os.path.join(images,image), outputdir)
            shutil.copy(os.path.join(pointclouds,pointcloud), outputdir)
            break
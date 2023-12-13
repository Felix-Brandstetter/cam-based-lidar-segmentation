
"""
Script for dconvert bayer image to RGB
Usage:
    1.Enter params
"""

#params
inputdir = "./data/Extracted-Data/images/"
outputdir = "./data/Extracted-Data/debayer"


import glob
import os
import cv2
import utils
count=0
files = sorted(glob.glob(os.path.join(inputdir,"*.png")))
utils.create_empty_dir(outputdir)
for file in files:
    imageRaw = cv2.imread(file, cv2.IMREAD_GRAYSCALE | cv2.IMREAD_ANYDEPTH)
    rgb = cv2.cvtColor(imageRaw, cv2.COLOR_BAYER_BG2RGB)
    cv2.imwrite(os.path.join(outputdir,"frame%008i.png" % count,),rgb)
    count=count +1 

print("Files converted")
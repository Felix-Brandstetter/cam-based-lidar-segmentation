import cv2
import numpy as np
img =cv2.imread("./data/Extracted-Data/debayer/frame00000546.png")


src = np.ones((6, 3))
src[:,1] = 2
src[:,2] = range(6) # source points

rvec = np.array([0,0,0], np.float) # rotation vector
tvec = np.array([0,0,0], np.float) # translation vector
fx = fy = 1.0
cx = cy = 0.0

cameraMatrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])
points2d ,useless= cv2.projectPoints(src, rvec, tvec, cameraMatrix, None)
print(points2d)
for n in range(len(points2d)):
    print(tuple(points2d[n].round().astype('int')))
    cv2.circle(img,tuple(points2d[n][0].round().astype('int')), 0, 2,-1)
cv2.imshow("st",img)
cv2.waitKey()
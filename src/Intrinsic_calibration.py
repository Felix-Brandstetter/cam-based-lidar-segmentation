import os
import cv2
import glob
import numpy as np
import os
import utils

"""
Script for intrinsic calibration via opencv
Usage:
    1. Enter params
    2. Press ENTER if corner are detected. Press any other key if corners are not detected
"""


#params
inputdir = "./data/Extracted-Data/images"
outputdir = "./data/Intrinsic-Calibration"
CHECKERBOARD = (11, 8)
undistord_imges = True

def intrinsic_calibration(inputdir,CHECKERBOARD,outputdir):
    utils.create_empty_dir(os.path.join(outputdir,"Parameters"))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objpoints = []
    imgpoints = []
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(
        -1, 2
    )
    #images = sorted(glob.glob("./data/Intrinsic-Calibration/Selected-Image-for-Intrinsic-Calibration/*.png"))
    images = iter(glob.glob(inputdir+"/*png"))
    while(len(objpoints)<15):
        fname = next(images)
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if ret == True:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

        cv2.imshow("img", img)
        k = cv2.waitKey()
        if k == 13: #Enter
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            print("Points safed")
        else:
            print("Points not safed")
    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    np.save(outputdir + "/Parameters/camera_matrix",mtx)

    np.save(outputdir + "/Parameters//dist",dist)
    np.save(outputdir +"/Parameters/rvecs",rvecs)
    np.save(outputdir + "/Parameters/tvecs",tvecs)
    
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print("Reprojectionerror: %s" %str(mean_error))
    return mtx,dist

def undistord_images(inputdir,camera_matrix,distCoeffs,outputdir):
    print("Undistord images ...")
    utils.create_empty_dir(os.path.join(outputdir,"Undistorded-Images"))
    count = 0
    newcameramtx = None
    for file in sorted(glob.glob(inputdir+"/*png")):

        
        img = cv2.imread(file)
        if newcameramtx is None:
            h,w = img.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distCoeffs, (w,h), 1, (w,h))
        undistorded_image = cv2.undistort(img, camera_matrix, distCoeffs, None, newcameramtx)
        cv2.imwrite(os.path.join(outputdir ,"Undistorded-Images/undistorted_frame%008i.png" % count),undistorded_image)
        count += 1

if __name__ == "__main__":
    camera_matrix, distCoeffs = intrinsic_calibration(inputdir,CHECKERBOARD,outputdir)
    if undistord_images:
        undistord_images(inputdir,camera_matrix,distCoeffs,outputdir)

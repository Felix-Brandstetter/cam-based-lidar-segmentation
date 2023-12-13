"""
Script for End to End calibration beetween Camera and Lidar (Intrinsic + Extrinsic)
Usage:
    1.Enter params
    2.Enter initial guess
    3.Start Script
    4. Pick 3D Point
        4.1. Select point with SHIFT + LEFT MOUSE CLICK
        4.2. Reserve selected point with SHIFT + RIGHT MOUSE CLICK
        4.3. Size selected point down with SHIFT + "?"
    5. Pick corresponding 2D point
        5.1. Select point with LEFT MOUSE CLICK
        5.2 Zoom in and out with MOUSE WHEEL
    6. Save points with ESC
"""
#params
picked_points_3D = 
picked_points_2D = 

#Initial Guess
    #Extrinsics
p = 0 # p: Roll (x-rotation)
q= 0 # q: Pitch (y-rotation)
r= 0 # r: Yaw (z-rotation)
oX= 0.95 # oX, oY, oZ: extrinsics' offset
oY= 0 
oZ = -0.35
    #Intrinsics
c= 1 # c: cam constant
m = 0 # m: scale difference
s = 0 #sheer
xH =0 # xH, yH: Principal point
yH = 0 

import os
import numpy as np
import scipy as sp
import scipy.optimize


def calLoss(X,x,p,q,r,oX,oY,oZ,c,m,s,xH,yH):
  x = np.transpose(x)
  #====EXTRINSICS====
  #Rotation
  Rx=np.array([[1.,0.,0.],[0.,np.cos(p),-np.sin(p)],[0.,np.sin(p),np.cos(p)]])
  Ry=np.array([[np.cos(q),0.,np.sin(q)],[0.,1.,0.],[-np.sin(q),0.,np.cos(q)]])
  Rz=np.array([[np.cos(r),-np.sin(r),0.],[np.sin(r),np.cos(r),0.],[0.,0.,1.]])
  R=Rx@Ry@Rz
  
  #Offset
  O=np.array([oX,oY,oZ])
  
  EXTR=np.column_stack((R,R@O))
  EXTR=np.row_stack((EXTR,np.array([0.,0.,0.,1.])))

  
  #====INTRINSICS=====
  #3D to image Plane
  PROJ=np.array([[c,0.,0.,0.],[0.,c,0.,0.],[0.,0.,1.,0.]])

  #Principal Point, Sheer, Scale
  PPSS=np.array([[1.,s,xH],[0.,1.+m,yH],[0.,0.,1]])
  
  #====LOSS====
  x_est=PPSS@PROJ@EXTR@np.transpose(X)
  weight=np.matrix([x_est[2]**(-1),x_est[2]**(-1),x_est[2]**(-1)])
  x_est=np.multiply(weight,x_est)
  return np.sum((x_est-x)@np.transpose(x_est-x))

def calLossWrapped(X,x):
    return lambda param: 10**0*calLoss(X,x,param[0],param[1],param[2],param[3],param[4],
                                 param[5],param[6],param[7],param[8],param[9],param[10])

#Calibration
def calibrateExtrinsics(X,x,p,q,r,oX,oY,oZ,c,m,s,xH,yH):
  x0=[p,q,r,oX,oY,oZ,c,m,s,xH,yH]
  opt=sp.optimize.minimize(calLossWrapped(X,x),x0,options={
    'maxiter':10**9,'gtol':10**(-10)},method='CG')
  print("Result of the Extrinsisic calibration" + str(opt.x))
  return opt.x
  

if __name__ == "__main__":
    #np.load()
    calibrateExtrinsics(picked_points_3D,picked_points_2D,p,q,r,oX,oY,oZ,c,m,s,xH,yH)



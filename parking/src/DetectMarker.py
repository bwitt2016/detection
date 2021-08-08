#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 29 14:52:34 2021

@author: brandon
"""

import cv2
import numpy as np
import os
import cv2.aruco as aruco
import math


#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0


#functions found on openCV turtorial https://learnopencv.com/rotation-matrix-to-euler-angles/
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])







cap=cv2.VideoCapture(0)
square=0.0254
marker_length=0.03700
dimensions=(6,9)

camera_matrix=np.array([[639.19703919  , 0.  ,       337.70929068],
 [  0,         637.64592654, 239.56944426],
 [  0.       ,    0.      ,     1.        ]])


distcoeff=np.array([[ 0.04226239 ,-0.56981459 , 0.0018108 ,  0.00523134 , 1.15592271]])



def findArucoMarkers(img,markerSize=6,totalMarkers=250,draw=True):
    imgGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    key=getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict=aruco.Dictionary_get(key)
    arucoParam=aruco.DetectorParameters_create()
    boxes,ids,rejected=aruco.detectMarkers(imgGray,arucoDict,parameters=arucoParam)
    
    if ids == None:
        return boxes, ids, None
    if draw and len(boxes)>0:
        aruco.drawDetectedMarkers(img,boxes)
        rvec,tvec,marker_points=aruco.estimatePoseSingleMarkers(boxes[0],square,camera_matrix,distcoeff)
        aruco.drawAxis(img,camera_matrix,distcoeff,rvec,tvec,marker_length)
        
        mat=np.array([[rvec[0][0][0]],[rvec[0][0][1]],[rvec[0][0][2]]])
        
        rod=np.matrix(cv2.Rodrigues(rvec)[0])
        
        
        r_matrix=rod.T
        roll,pitch,yaw=rotationMatrixToEulerAngles(R_flip*r_matrix)
        orientation=[roll,pitch,yaw]
        print("MARKER POSITION:" , math.degrees(roll),math.degrees(pitch),math.degrees(yaw))
    else:
        orientation=None
        pitch=None
        
        
    return boxes,ids,pitch

def calc_center(boxes):
    x=[]
    y=[]
    for point in boxes[0][0]:
       x.append(point[0])
       y.append(point[1])
    xs=0.5*(max(x)+min(x))
    ys=0.5*(max(y)+min(y))
    
    
    return xs,ys


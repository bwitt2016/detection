#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
import time
from DetectMarker import *
import cv2.aruco as aruco


pub=rospy.Publisher('cmd_vel',Twist)
import copy
flag=0
low_red=np.array([0,50,160])
high_red=np.array([10,255,255])

def delay(s):
    for sec in range(s):
     print(sec+1)
     for x in range(12000000):
        x=x
class tbot3(object):
    def __init__(self):

        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
        rospy.Subscriber('/scan',LaserScan,self.callback2)
        self.angle=None
        
        self.front=None
        self.L=None
        self.x=None
        self.y=None
        
        
    def callback2(self,msg):
            self.front=min([msg.ranges[0],msg.ranges[1],msg.ranges[359],msg.ranges[358],msg.ranges[2]])
    def callback(self,msg):
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        o=msg.pose.pose.orientation
        o_lst=[o.x,o.y,o.z,o.w]
        (roll,pitch,yaw)=euler_from_quaternion(o_lst)
        self.angle=(yaw/math.pi)*180
        
        
        
            
    
    def get_values(self):
       
       return self.angle,self.front

class parking(object):
    #bridge object and image sub declared
    def __init__(self):
        self.bridge_object=CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.park_callback)
        self.tbot3=tbot3()
        self.flag=0
        self.d=None
        self.L=None
        self.theta=None
        self.turn_angle=None
        self.side=None
        self.o=None
        self.origin=None
        self.pos=None
        self.boxes=None
        self.ids=None
        self.marker=None
        
        
        
    #callback (looping) function
    def park_callback(self,data):
        
        try:
            
            #raw image in bgr
            cv_image=self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
            #size variables for image
            height,width,channels=cv_image.shape
            median=width/2
            hsv_frame=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
            
            #mask out nonyellows
            red=cv2.inRange(hsv_frame,low_red,high_red)
            m=cv2.moments(red,False)
            
            self.boxes,self.ids,self.marker=findArucoMarkers(cv_image)
                
            try:
                #center of moments
               
                x,y=calc_center(boxes)
                cv2.circle(frame,(int(x),int(y)),10,(0,0,255),-1)
                
            except IndexError:
                x,y=0,0
                #wx,wy=width,width
           
            if self.flag==0:
                 self.flag=rotate(x,width/2)
                 
                 
                 
            if self.flag==1:
                if self.tbot3.front!=None:
                         
                     self.d=self.tbot3.front
                     
                     self.theta=self.marker
                     self.o=copy.deepcopy(self.tbot3.angle)
                     self.turn_angle=90-self.theta
                     
                     self.L=self.d*math.sin(math.radians(self.theta))
                     self.origin=[copy.deepcopy(self.tbot3.x),copy.deepcopy(self.tbot3.y)]
                     
                     
                     if self.marker>0:
                         self.side="right"
                     else:
                         self.side="left"
                     print("found marker")
                     print("orientation: ",self.o)
                     print("distance to maker: ",self.d)
                     print("distance of L: ", self.L)
                     print("theta: ",self.theta)
                     print("Angle: ", 90-self.theta)
                     print("side: ",self.side)
                     self.flag=2
                else:
                    self.flag=0
                     
            if self.flag==2:
                
                self.flag=turn(self.o,self.tbot3.angle,self.turn_angle,self.side)
                
                
                
            if self.flag==3:
                self.pos=[copy.deepcopy(self.tbot3.x),copy.deepcopy(self.tbot3.y)]
                self.flag=move_along_L(self.L, self.origin, self.pos)
            if self.flag==4:
                self.flag=rotate(x,width/2,self.side)+4
            if self.flag==5:
                self.flag=follow(x,width/2,self.tbot3.front)
            
            
                
            cv2.imshow("Image Window",cv_image)
            
            cv2.waitKey(1)
            
            
        except CvBridgeError as e:
            print(e)
        

def rotate(x,med,side=None):
    
    move=Twist()
    if abs(x-med)<=5:
        move.angular.z=0
        
        pub.publish(move)
        time.sleep(3)
        return 1
    z=0.4
    if side!=None:  
        if side=="left":
            z=z*(-1)
    move.angular.z=z
    
    pub.publish(move)
    return 0

#takes current direction and rotates degrees to one side
#origin is the original orientation
#direction is the current orientation
#degrees is the goal
#side determines which side we turn
def turn(initial,direction,degrees,side):
    move=Twist()
    
    
    difference=abs(initial)-abs(direction)
    
    if difference>=degrees-5:#tendency to overestimate
        
        move.angular.z=0
        pub.publish(move)
        return 3
    z=0.4
    if side=="right":
        z=z*(-1)
    move.angular.z=z
    pub.publish(move)
    return 2

#move until distance between current and origin is 
def move_along_L(L,origin,current):
    move=Twist()
    dist=distance(origin,current)
    if dist>L:
        move.linear.x=0
        pub.publish(move)
        return 4
    move.linear.x=0.4
    pub.publish(move)
    return 3
    
def distance(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)


#gets angular velocity based on percent change
def get_av(offset):
    if abs(offset)<=5:
        return 0.01
    if abs(offset)<=15:
        return 0.05
    return 0.1

def follow(x,med,front):
    move=Twist()
    print(x,med)
    offset=(med-x)/int(med*2/100)
    z=get_av(offset)
    x=0.25
    if x==0 or front=="inf":
        move.linear.x=0
        move.angular.z=0
        pub.publish(move)
        return 0
    print(front)
    if float(front)<0.24:
        print("Parking complete")
        move.linear.x=0
        move.angular.z=0
        pub.publish(move)
        return 10
    
    
    print("OFFSET: ",offset)
    
    if offset<0:#if point is to the right
        z=z*(-1)#turn right
    move.linear.x=x
    move.angular.z=z
    print("angular speed: ",z)
    print("linear speed: ",x)
    pub.publish(move)
    return 5
if __name__ == '__main__':
    
    #run detect_line 
    detect=parking()
    #node
    rospy.init_node('parking')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")


        


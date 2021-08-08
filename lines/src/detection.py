#! /usr/bin/env python2

"""
Created on Wed Jul 21 09:59:49 2021

@author: brandon
"""
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
w,h=200,275
median=w/2


low_yellow=np.array([20,100,220])
high_yellow=np.array([65,255,255])

low_white=np.array([0,0,240])
high_white=np.array([255,30,255])

pts1=np.float32([[0,445],[639,445],[0,479],[639,479]])
pts2=np.float32([[0,0],[w,0],[0,h],[w,h]])



matrix=cv2.getPerspectiveTransform(pts1,pts2)




prev=[0,0]

pub=rospy.Publisher('cmd_vel',Twist)


class Detect_line(object):
    #bridge object and image sub declared
    def __init__(self):
        self.bridge_object=CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw",Image,self.camera_callback)
    
    #callback (looping) function
    def camera_callback(self,data):
        try:
            switch=[1,1]
            
            #raw image in bgr
            cv_image=self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
            #size variables for image
            height,width,channels=cv_image.shape
            median=width/2
            
            #crop of image
            crop_image=cv_image[height/2+80:height][1:width]
            #convert crop image to HSV
            warped=cv2.warpPerspective(cv_image,matrix,(w,h))
            hsv_frame=cv2.cvtColor(warped,cv2.COLOR_BGR2HSV)
            
            #mask out nonyellows
            yellow=cv2.inRange(hsv_frame,low_yellow,high_yellow)
            #white mask
            white=cv2.inRange(hsv_frame,low_white,high_white)
            
            gray_frame=cv2.cvtColor(warped,cv2.COLOR_BGR2GRAY)
            edges=cv2.Canny(gray_frame,30,150,apertureSize=3)
           
            #y_gray=cv2.cvtColor(yellow,cv2.COLOR_HSV2GRAY)
            y_edges=cv2.Canny(yellow,30,150,apertureSize=3)
            #=cv2.cvtColor(white,cv2.COLOR_BGR2GRAY)
            w_edges=cv2.Canny(white,30,150,apertureSize=3)
            
            
            
            
            wm=cv2.moments(w_edges,False)
            ym=cv2.moments(y_edges,False)
            
            try:
                #center of moments
               
                wx,wy=wm['m10']/wm['m00'],wm['m01']/wm['m00']
                
                
            except ZeroDivisionError:
                #if no moment exists, then just post in center
               
                #print("white out of bounds")
                switch[1]=0
                #wx,wy=width,width
                
            try:
                #center of moments
                cx,cy=ym['m10']/ym['m00'],ym['m01']/ym['m00']
                
                
            except ZeroDivisionError:
                #if no moment exists, then just post in center
                #print("Yellow out of bounds")
                switch[0]=0
                #cx,cy=0,0
                
            
            
        except CvBridgeError as e:
            print(e)
        
        
        
        cv2.circle(cv_image,(pts1[0][0],pts1[0][1]),10,(0,60,250),-1)
        cv2.circle(cv_image,(pts1[1][0],pts1[1][1]),10,(0,60,250),-1)
        cv2.circle(cv_image,(pts1[2][0],pts1[2][1]),10,(0,60,250),-1)
        cv2.circle(cv_image,(pts1[3][0],pts1[3][1]),10,(0,60,250),-1)
        
       
        
        
        if switch[1]==1 and sum(switch)==1:
            cv2.circle(warped,(int(wx),int(20)),10,(0,0,255),-1)
            cv2.circle(warped,(int(wx/2),int(40)),10,(0,250,35),-1)
            print(wx)
            follow(wx/2-20,w/2)
           
        
        if switch[0]==1 and sum(switch)==1:
            cv2.circle(warped,(int(cx),int(20)),10,(0,0,255),-1)
            cv2.circle(warped,(int((cx+199))/2,int(40)),10,(0,250,35),-1)
            print(cx)
            follow(cx+w/2+20,w/2)
        
            
        if sum(switch)==2:
            x=(wx+cx)/2
            cv2.circle(warped,(int(x),int(10)),10,(0,200,100),-1)
            follow(x,w/2)
            print(w/2-wx);print(w/2-cx)
        #when you do not detect either line
        if sum(switch)==0:
            follow(-1,None)
        
        print(switch)
        
        
        cv2.imshow("Image Window",cv_image)
        cv2.imshow("YELLOW EDGES",yellow)
        cv2.imshow("WHITE EDGES",white)
        cv2.imshow("warp",warped)
        cv2.waitKey(1)
        #h,w,c=warped.shape
        


#calculates difference of x and median
#the median is 
def percent_change(val,median):
    #define percentage 
    increment=int((median*2)/100)
    #find difference
    diff=median-val
    #find percent change
    percent_change=diff/increment
    return percent_change
#gets angular velocity based on percent change
def get_av(offset):
    if abs(offset)<=10:
        return 0.05
    if abs(offset)<=20:
        return 0.2
    if abs(offset)<=25:
        return 0.4
    if abs(offset)<=30:
        return 0.8
    if abs(offset)<=70:
        return 1.6
    return 1.8


#executes follow line
def follow(x,med):
    move=Twist()
    if(x==-1):
        move.linear.x=0
        move.angular.z=0
        pub.publish(move)
    offset=percent_change(x,med)
    print(offset)
    z=get_av(offset)
    if offset<0:
        z=z*(-1)
    if offset>10:
        move.linear.x=0.25
    else:
        move.linear.x=0.25
    move.angular.z=z
    print("angular speed: ",z)
    pub.publish(move)


        
def main():
    #run detect_line 
    detect=Detect_line()
    #node
    rospy.init_node('lines')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
        
    #close windows on shut down
    
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main()
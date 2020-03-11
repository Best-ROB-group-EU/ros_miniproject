#!/usr/bin/env python
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import LaserScan #Published message from lidar

img_size = 800                  #Self explanatory
bg_col = 255                    #Background colour
edge_col = 0                    #Edge colour
edge_kernel = 2                 #Edge thickness
centering = img_size/2          #Center coordinates for image
dis_scale = 30                  #Measured distance scalar: 1 meter = 30 pixel

img = np.zeros([img_size,img_size,1])                        #Creating blank image

img[:,:,0] = np.ones([img_size,img_size])*bg_col/255         #Colour background - 0 colour channels for greyscale

def callback(ranges):                                        #Make callback to range[] member of LaserScan struct 
    dist = ranges.ranges
    for x in range(0,360):                                   #Each array contains 360 entries, one for each degree measured
        ang = x
        dis = dist[x]
        
        if dis < 16:                                         #Check for infinite value for too close/distant objects
            s_dis = dis * dis_scale                          #Scale distance for visibility
        
            rad = ang*(math.pi/180)                          #Convert degree to radian
            xpos = s_dis * math.cos(rad)                     #Polar to cartesian coordinate
            ypos = s_dis * math.sin(rad)
        
            rx_pos = round(xpos,0)
            ry_pos = round(ypos,0)
            x_pos = int(rx_pos)                              #Round and convert to integers because there are no float pixels
            y_pos = int(ry_pos)
            for x in range(x_pos-edge_kernel,x_pos+edge_kernel):        #Colour coordinate + pixels around them
                cent_x = int(centering+x)                               #Determine coordinate relative to center of image
                cent_y = int(centering+y_pos)
                img[cent_x, cent_y, 0] = edge_col/255
                for y in range(y_pos-edge_kernel,y_pos+edge_kernel):
                    cent_y = int(centering+y)
                    img[cent_x, cent_y,0] = edge_col/255
                cv2.imwrite('scan.jpg', img)
                cv2.imshow("image", img)
                cv2.waitKey(1)                                          #Update image every 1 millis

def listener():
    rospy.init_node('listener', anonymous = True)
    
    rospy.Subscriber("/scan", LaserScan, callback)       #RPLidar publishes topic /scan, therefore subscribe to this topic and run call back for LaserScan
    
    rospy.spin()                                     #Spin() keeps things going

listener()
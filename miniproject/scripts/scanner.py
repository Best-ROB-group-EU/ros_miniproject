#!/usr/bin/env python
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import LaserScan #Published message from lidar
import timeit


img_size = 800                  #Self explanatory
bg_col = 255                    #Background colour
edge_col = 0                    #Edge colour
edge_kernel = 1                 #Edge thickness
centering = int(img_size/2)          #Center coordinates for image
dis_scale = 60                  #Measured distance scalar: 1 meter = 60 pixel


#To continously paint data on the same image uncomment below and comment them out in the callback
#img = np.zeros([img_size,img_size,1])                        #Creating blank image
#img[:,:,0] = np.ones([img_size,img_size])*bg_col/255         #Colour background - 0 colour channels for greyscale

lidar_l = int(centering-2)
lidar_h = int(centering+2)

#for x in range(lidar_l,lidar_h):                             #Paint lidar location on map in grey
#    img[x,centering,0] = 127/255
#    for y in range(lidar_l,lidar_h):
#        img[x,y,0] = 127/255

#cm = int(round(100/80,0))                                    #Paint tape measure for testing
#for x in range(0,30):
#    distance = int(x*(10*cm))
#    for y in range(0,360):
#        angle = y
#        radian = angle*(math.pi/180)
#        pos_x = int(distance*math.cos(radian))
#        pos_y = int(distance*math.sin(radian))
#        c_pos_x = int(centering+pos_x)
#        c_pos_y = int(centering+pos_y)
#        img[c_pos_x,c_pos_y] = 200/255

def callback(ranges):                                        #Make callback to range[] member of LaserScan struct
    dist = ranges.ranges

    #Comment here to continously paint data
    img = np.zeros([img_size,img_size,1])                        #Creating blank image
    img[:,:,0] = np.ones([img_size,img_size])*bg_col/255         #Colour background - 0 colour channels for greyscale
    for x in range(lidar_l,lidar_h):                             #Paint lidar location on map in grey
        img[x,centering,0] = 127/255
        for y in range(lidar_l,lidar_h):
            img[x,y,0] = 127/255

    for x in range(0,360):                                   #Each array contains 360 entries, one for each degree measured
        ang = x
        dis = dist[x]

        if dis < 16:                                         #Check for infinite value for too close/distant objects
            s_dis = dis * dis_scale                          #Scale distance for visibility

            rad = ang*(math.pi/180)                          #Convert degree to radian
            xpos = s_dis * math.cos(rad)                     #Polar to cartesian coordinate
            ypos = s_dis * math.sin(rad)

            cent_x = int(centering+xpos)
            cent_y = int(centering+ypos)
            img[cent_x, cent_y, 0] = edge_col/255

            #for x in range(cent_x-edge_kernel,cent_x+edge_kernel):        #Paint coordinate + pixels around them (larger edge kernel)
            #    img[x, cent_y, 0] = edge_col/255
            #    for y in range(cent_y-edge_kernel,cent_y+edge_kernel):
            #        img[x, y,0] = edge_col/255

    cv2.imwrite('scan.jpg', img)
    cv2.imshow("image", img)
    cv2.waitKey(1)

def listener():
    rospy.init_node('listener', anonymous = True)

    rospy.Subscriber("/scan", LaserScan, callback)       #RPLidar publishes topic /scan, therefore subscribe to this topic and run call back for LaserScan

listener()

rospy.spin()                                     #Spin() keeps things going

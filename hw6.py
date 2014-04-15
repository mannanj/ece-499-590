#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 640
ny = 480

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
v = ach.Channel(ROBOT_CHAN_VIEW)
v.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0
timeupdated=0
#i added this to be able to write to file without unnecessarily opening the file every time the loop runs
f = open('samples', 'wr')
f1 = open('error_samples', 'wr')
f2 = open('time', 'wr')

print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    img = np.zeros((newx,newy,3), np.uint8)
    c_image = img.copy()
    vid = cv2.resize(c_image,(newx,newy))
    [status, framesize] = v.get(vid, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vid,(nx,ny))
        img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl", img)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # img        = cv image in BGR format

    ts=tim.sim[0]
    ref.ref[0] = -0.4
    ref.ref[1] = 0.4

    # Sets reference to robot
    r.put(ref);

    # Sleeps
    #time.sleep(0.1)   

    # Convert BGR to HSV, define range of colors, for only blue, this to get the moments
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([110, 50, 50], dtype=np.uint8)
    upper_blue = np.array([130,255,255], dtype=np.uint8)
    maskb = cv2.inRange(hsv, lower_blue, upper_blue)

    #gets moment & centroid and prints earlier blue mask converted to black&white
    moments1 = cv2.moments(maskb)
    if moments1['m00'] > 0:
        frame1 = cv2.rectangle(maskb,(300,100),(400,500),(0,255,0))
        frame2 = maskb[100:500,300:400]
        cv2.imshow('Mask',maskb)
        cnt=moments1['m00']
        if cnt != 0.0:
            cx = moments1['m10']/moments1['m00']
            print "cx", 
            cy = moments1['m01']/moments1['m00']
            centroid = (cx, cy)
            #print "Centroid = ", centroid

    #check if centroid is in frame and turns towards direction of the centroid
    if (moments1['m00'] >0):
        if (cx<=324):
            ref.ref[0] = 0.1
            ref.ref[1] = -0.1    
            r.put(ref);   
            print "object on left ", centroid
            if (cx<=309):
                ref.ref[0] = 0.35
                ref.ref[1] = -0.35  
                r.put(ref);    
            if (cx<=281):
                ref.ref[0]= 0.65
                ref.ref[1]= -0.65    
                r.put(ref);      
        elif (cx>=325):
            ref.ref[0]= -0.1
            ref.ref[1]= 0.1
            r.put(ref);
            print "object greater on right = ", centroid
            if (cx>=340):
                ref.ref[0]= -0.35
                ref.ref[1]= 0.35
                r.put(ref);
                print "object greater on right = ", centroid
                if (cx>=375):
                    ref.ref[0]= -0.65
                    ref.ref[1]= 0.65
                    r.put(ref);
                    print "object greater on right = ", centroid

    #get sim times and calculate difference+error
    ts2=tim.sim[0]
    difference=abs(ts-ts2) 
    while (0.1 > difference):   
        [status, framesize] = t.get(tim, wait=False, last=True)
        ts2=tim.sim[0]
        difference=abs(ts-ts2)
    #write data to files, and get updated time
    if (moments1['m00'] > 0): 
	    #get updated time
        timeupdated=timeupdated+difference
        error = (abs(320-cx)/320)*100
        print "error", error
        f.write(str(cx))
        f.write('\n')
        f1.write(str(error))
        f1.write('\n')
        f2.write(str(timeupdated))
        f2.write('\n')

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------


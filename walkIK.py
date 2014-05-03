#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2013, Daniel M. Lofaro
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


import termios, fcntl, sys, os #To allow for keyboard commands
import hubo_ach as ha
import ach
import sys
import time
import math
from ctypes import *

# Open Hubo-Ach feed-forward and feed-back (reference and state) channels
s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
r = ach.Channel(ha.HUBO_CHAN_REF_NAME)
#s.flush()
#r.flush()

# feed-forward will now be refered to as "state"
state = ha.HUBO_STATE()

# feed-back will now be refered to as "ref"
ref = ha.HUBO_REF()

# Get the current feed-forward (state) 
[statuss, framesizes] = s.get(state, wait=False, last=False)

fd = sys.stdin.fileno()

oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)


#initialize all joints to 0 
RHR = 0; 
LHR = 0; 
RHP = 0;
LHP = 0;
RKN = 0; 
LKN = 0; 
RAP = 0; 
LAP = 0; 
RAR = 0; 
LAR = 0; 
RSP = 0;  
LSP = 0;
RSR = 0;
LSR = 0;
RWP = 0;
LWP = 0;
REB = 0;
LEB = 0;
RSR = 0;
LSR = 0;
RF5 = 0;
LF5 = 0;
mytheta_i1 = 0;
mytheta_i2 = 0;
mytheta_i3 = 0;
mytheta_r1 = 0;
mytheta_r2 = 0;
mytheta_r3 = 0;
mytheta_l1 = 0;
mytheta_l2 = 0;
mytheta_l3 = 0;
speed = 40;
speed1 = 100;
speed2 = 15;
    
#neutral angles for the arms (no kinematics needed here)
def AnglesArmsOut():
    thet1=.2
    thet2=0.4
    thet3=0.4
    return (thet1, thet2, thet3)

#neutral angles for the knees (no kinematics needed here)
def AnglesKneeBend():
    thet1=1.2
    thet2=1.2
    thet3=0.4
    return (thet1, thet2, thet3)

#calculate IK angles for intial foot position
def InverseKinematicsInitial():
    xe=0.0
    ye=0.0
    l1 = .30003
    l2 = .30038
    l3 = .09497
    phi_e = 0
    xw = xe - math.acos(phi_e)*l3
    yw = ye - math.acos(phi_e)*l3
    xw_sq = math.pow(xw,2);
    yw_sq = math.pow(yw,2);
    alpha = math.atan(yw/xw);
    gamma1 = math.pow(l1,2)-math.pow(l2,2) + xw_sq + yw_sq;
    gamma2 = 2*l1*math.sqrt(xw_sq+yw_sq);
    gamma = math.acos(gamma1/gamma2);
    Beta1 = math.pow(l1,2)+math.pow(l2,2)-xw_sq-yw_sq;
    Beta2 = 2*l1*l2;
    Beta = math.acos(Beta1 / Beta2);
    radius = math.sqrt (math.pow(l1, 2) + math.pow(l2,2));
    theta2 = 3.14 - Beta;
    theta1 = alpha - gamma;
    theta3 = 0 - theta1 - theta2;
    print "Initial Angles of legs: ", theta1, theta2, theta3
    return (theta1, theta2, theta3)
InverseKinematicsInitial();
#Get initial angle states, to subtract from the new leg states as an offset 
mytheta_i1, mytheta_i2, mytheta_i3 = InverseKinematicsInitial(); 

#calculate IK equations for lifting leg
def InverseKinematicsL():
    xe=0.0
    ye=0.38
    l1 = .30003
    l2 = .30038
    l3 = .09497
    phi_e = 0
    xw = xe - math.acos(phi_e)*l3
    yw = ye - math.acos(phi_e)*l3
    xw_sq = math.pow(xw,2);
    yw_sq = math.pow(yw,2);
    alpha = math.atan(yw/xw);
    gamma1 = math.pow(l1,2)-math.pow(l2,2) + xw_sq + yw_sq;
    gamma2 = 2*l1*math.sqrt(xw_sq+yw_sq);
    gamma = math.acos(gamma1/gamma2);
    Beta1 = math.pow(l1,2)+math.pow(l2,2)-xw_sq-yw_sq;
    Beta2 = 2*l1*l2;
    Beta = math.acos(Beta1 / Beta2);
    rad = math.sqrt (math.pow(l1, 2) + math.pow(l2,2));
    theta2 = 3.14 - Beta;
    theta1 = alpha - gamma;
    theta3 = 0 - theta1 - theta2;
    return (theta1, theta2, theta3)

#calculate IK equations for lifting leg
def InverseKinematicsR():
    xe=0.0
    ye=0.05
    l1 = .30003
    l2 = .30038
    l3 = .09497
    rad2degrees = 3.14*180;
    phi_e = 0 * rad2degrees;
    xw = xe - math.acos(phi_e)*l3
    yw = ye - math.acos(phi_e)*l3
    xw_sq = math.pow(xw,2);
    yw_sq = math.pow(yw,2);
    alpha = math.atan(yw/xw);
    gamma1 = math.pow(l1,2)-math.pow(l2,2) + xw_sq + yw_sq;
    gamma2 = 2*l1*math.sqrt(xw_sq+yw_sq);
    gamma = math.acos(gamma1/gamma2);
    Beta1 = math.pow(l1,2)+math.pow(l2,2)-xw_sq-yw_sq;
    Beta2 = 2*l1*l2;
    Beta = math.acos(Beta1 / Beta2);
    rad = math.sqrt (math.pow(l1, 2) + math.pow(l2,2));
    theta2 = 3.14 - Beta;
    theta1 = alpha - gamma;
    theta3 = 0 - theta1 - theta2;
    return (theta1, theta2, theta3)


#This is to initially bend the knees, these angles could be decreased
mytheta_r1, mytheta_r2, mytheta_r3 = AnglesKneeBend(); 
mytheta_l1=mytheta_r1;
mytheta_l2=mytheta_r2;
mytheta_l3=mytheta_r3;
print "Bending Knees to Initial State"
print "Desired Right leg angles in rad (RHP RKN RAP)  ", -1*mytheta_r1, -mytheta_r2, -1*mytheta_r3
print "Desired Left leg angles in rad (LHP LKN LAP) ", -1*mytheta_l1, -1*mytheta_l2, -1*mytheta_l3
#set them slowly, set them in increments of "speed"
for i in range (0, speed):
    RHP = RHP + -1*mytheta_r1/speed;
    ref.ref[ha.RHP] = RHP;
    r.put(ref)
    LHP = LHP + -1*mytheta_l1/speed;
    ref.ref[ha.LHP] = LHP;
    r.put(ref)
    RKN = RKN + mytheta_r2/speed;
    ref.ref[ha.RKN] = RKN;
    r.put(ref)
    LKN = LKN + mytheta_l2/speed;
    ref.ref[ha.LKN] = LKN;
    r.put(ref)
    RAP = RAP + -1*mytheta_r3/speed;
    ref.ref[ha.RAP] = RAP;
    r.put(ref)
    LAP = LAP + -1*mytheta_l3/speed;
    ref.ref[ha.LAP] = LAP;
    r.put(ref)
    #print "Right Position  (RHP RKN RAP)  ", RHP, RKN, RAP
    #print "Left Positions  (LHP LKN LAP) ", LHP, LKN, LAP
    time.sleep(0.5)

#bring out arms to improve balance
mytheta_r1, mytheta_r2, mytheta_r3 = AnglesArmsOut(); 
mytheta_l1=mytheta_r1;
mytheta_l2=mytheta_r2;
mytheta_l3=mytheta_r3;
print "Bending Arms to Neutral State"
print "Right arm angles in rad (RSP REB RWP) ", -1*mytheta_r1, -1*mytheta_r2, -1*mytheta_r3
print "Left arm angles in rad (LSP LEB LWP) ", -1*mytheta_r1, -1*mytheta_r2, -1*mytheta_r3
#set them slowly
for i in range (0, speed):
    RSP = RSP + -1*mytheta_r1/speed;
    ref.ref[ha.RSP] = RSP;
    r.put(ref)
    LSP = LSP + -1*mytheta_l1/speed;
    ref.ref[ha.LSP] = LSP;
    r.put(ref)    
    REB = REB + -1*mytheta_r2/speed;
    ref.ref[ha.REB] = REB;
    r.put(ref)
    LEB = LEB + -1*mytheta_l2/speed;
    ref.ref[ha.LEB] = LEB;
    r.put(ref)
    RWP = RWP + -1*mytheta_r3/speed;
    ref.ref[ha.RWP] = RWP;
    r.put(ref)
    LWP = LWP + -1*mytheta_l3/speed;
    ref.ref[ha.LWP] = LWP;
    r.put(ref)
    #print "Right Position  (RSP REB RWP)  ", RSP, REB, RWP
    #print "Left Positions  (LSP LEB LWP)  ", LSP, LEB, LWP
    time.sleep(0.25)

#Moving legs for walking motion
#First bend knees to the right
print "Moving Knee to the Right"
for i in range (0, 30):
    RHR = RHR + 0.005;
    ref.ref[ha.RHR] = RHR;
    r.put(ref)
    LHR = LHR + 0.005;
    ref.ref[ha.LHR] = LHR;
    r.put(ref)
    RAR = RAR - 0.005;
    ref.ref[ha.RAR] = RAR;
    r.put(ref)
    LAR = LAR - 0.005;
    ref.ref[ha.LAR] = LAR;
    r.put(ref)
    #print "Right Position  (RHR RAR)  ", RHR, RAR
    #print "Left Positions  (LHR LAR) ", LHR, LAR
    time.sleep(0.5)
print "Right Position  (RHR RAR)  ", RHR, RAR
print "Left Positions  (LHR LAR) ", LHR, LAR

#now Lift the right Leg, by only moving hip forward
mytheta_r1, mytheta_r2, mytheta_r3 = InverseKinematicsR(); 
difference = mytheta_r1 + mytheta_i1; #get the actual RHP
difference = -1*difference/3.14 #divide by PI to normalize to Hubo-Ach
print "Only the hip will change. New (RHP)", difference, "Old (RHP)", RHP
print "Difference =", difference
print "Right Leg Angles (RHP RKN RAP) ", mytheta_r1, mytheta_r2, mytheta_r3
#needs to be converted
for j in range (0, speed):
    RHP = RHP + (difference/speed);
    ref.ref[ha.RHP] = RHP;
    r.put(ref)
    time.sleep(0.5)
    #print "Right Position  (RHP RKN RAP)  ", RHP, RKN, RAP
    #print "Left Positions  (LHP LKN LAP) ", LHP, LKN, LAP

#bring left knee back a bit to leverage momentum to right foot
for j in range (0, speed2):
    RKN = RKN - (0.3/speed2);
    ref.ref[ha.RKN] = RKN;
    r.put(ref)
    time.sleep(0.5)
    print "Left Knee Back (LKN)", RKN
    if j>=(speed2)/2:
        LSP = LSP + .2/speed;
        ref.ref[ha.LSP] = LSP;
        r.put(ref)   
        LEB = LEB + .2/speed;
        ref.ref[ha.LEB] = LEB;
        r.put(ref)
        LWP = LWP + .3/speed;
        ref.ref[ha.LWP] = LWP;
        r.put(ref)
        time.sleep(0.25)
        print "Left Arm Back (LSP, LEB, LWP)", LSP, LEB, LWP
    if j>=speed2/2:
        LKN = LKN - (0.3/speed2);
        ref.ref[ha.LKN] = LKN;
        r.put(ref)
        time.sleep(0.5)   
        print " LKN going back (LKN)", LKN
mytheta_l1, mytheta_l2, mytheta_l3 = InverseKinematicsL(); 
print "Left Leg Angles (RHP RKN RAP)", mytheta_l1, mytheta_l2, mytheta_l3

# Close the connection to the channels
r.close()
s.close()

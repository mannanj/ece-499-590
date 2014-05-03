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

RHY = 0; # "#" means used in commands below
LHY = 0; # 
RHR = 0; # 
LHR = 0; #
RHP = 0;
LHP = 0;
RKN = 0; #
LKN = 0; #
RAP = 0; #
LAP = 0; #
RAR = 0; 
LAR = 0; 
WST = 0;
RSP = 0; #
RSR = 0; #
LSP = 0;
LSR = 0;
RWP = 0;
REB = 0;
RSR = 0;
LWP = 0;
LEB = 0;
LSR = 0;
RF5 = 0; # Right Finger
LF5 = 0; # Right Finger

time.sleep(2)

print "balancing via arms"
#bring out arms to improve balance
for i in range (0, 40):
    RWP = RWP-0.01;
    ref.ref[ha.RWP] = RWP;
    r.put(ref)
    LWP = LWP-0.01;
    ref.ref[ha.LWP] = LWP;
    r.put(ref)
    REB = REB-0.01;
    ref.ref[ha.REB] = REB;
    r.put(ref)
    LEB = LEB-0.01;
    ref.ref[ha.LEB] = LEB;
    r.put(ref)
    RSR = RSR-0.01;
    ref.ref[ha.RSR] = RSR;
    r.put(ref)
    LSR = LSR+0.01;
    ref.ref[ha.LSR] = LSR;
    r.put(ref)
    #RSR = RSR-0.02;
    #ref.ref[ha.RSR] = RSR;
    #r.put(ref)
    RSP = RSP - 0.009;
    ref.ref[ha.RSP] = RSP;
    r.put(ref)
    LSP = LSP - 0.009;
    ref.ref[ha.LSP] = LSP;
    r.put(ref)    
    RF5 = RF5 + 1; # Right Finger
    ref.ref[ha.RF5] = RF5;
    r.put(ref)
    LF5 = LF5 + 1; # Right Finger
    ref.ref[ha.LF5] = LF5;
    r.put(ref)
    time.sleep(0.25)

print "bending knees"
#now bend knees - balance at bended knees
for i in range (0, 40):
    RHP = RHP-0.03;
    ref.ref[ha.RHP] = RHP;
    r.put(ref)
    LHP = LHP-0.03;
    ref.ref[ha.LHP] = LHP;
    r.put(ref)
    RKN = RKN+0.03;
    ref.ref[ha.RKN] = RKN;
    r.put(ref)
    LKN = LKN+0.03;
    ref.ref[ha.LKN] = LKN;
    r.put(ref)
    RAP = RAP-0.01;
    ref.ref[ha.RAP] = RAP;
    r.put(ref)
    LAP = LAP-0.01;
    ref.ref[ha.LAP] = LAP;
    r.put(ref)
    time.sleep(0.5)

try:
    print "Press key: 'L' for left leg movement, or 'R' for right leg"
    while 1:
        try:
            c = sys.stdin.read(1); 
	    if c == 'l':
	    #bend knees to right
		for i in range (0, 30):
		    RHR = RHR+0.005;
		    ref.ref[ha.RHR] = RHR;
		    r.put(ref)
		    LHR = LHR+0.005;
		    ref.ref[ha.LHR] = LHR;
		    r.put(ref)
		    RAR = RAR-0.005;
		    ref.ref[ha.RAR] = RAR;
		    r.put(ref)
		    LAR = LAR-0.005;
		    ref.ref[ha.LAR] = LAR;
		    r.put(ref)
		    time.sleep(0.5)
		    print "moving knees to the right, slowly", i
		    #lift left (opposite) leg
		    if (i==29):
		        for j in range (0, 15):
			    LHP = LHP-0.03;
			    ref.ref[ha.LHP] = LHP;
			    r.put(ref)
			    time.sleep(0.5)
			    print "lifting left leg through pitch",j  
			#bend knee to center
		for i in range (0, 30):
		    RHR = RHR-0.005;
		    ref.ref[ha.RHR] = RHR;
		    r.put(ref)
		    LHR = LHR-0.005;
		    ref.ref[ha.LHR] = LHR;
		    r.put(ref)
		    RAR = RAR+0.005;
		    ref.ref[ha.RAR] = RAR;
		    r.put(ref)
		    LAR = LAR+0.005;
		    ref.ref[ha.LAR] = LAR;
		    r.put(ref)
		    time.sleep(0.5)
		    print "moving knees to the center, slowly", i
		    #bring right leg out
		    if (i==15):
			for j in range (0, 15):
				RHP = RHP-0.03;
				ref.ref[ha.LHP] = LHP;
				r.put(ref)
				RKN = RKN-0.03;
				ref.ref[ha.RHP] = RHP;
				r.put(ref) 
				time.sleep(0.5)
				print "bring out right leg, slowly",j
				#left knee back a bit, maybe change this concurrent with right leg above
				if (i==30):
				    for j in range (0, 15):
			      	        LKN = LKN+0.03;
					ref.ref[ha.LHP] = LHP;
					r.put(ref) 
				        time.sleep(0.5)
				        print "bring back left knee, slowly",j						
    	    if c == 'r': 
		#bend knee to right side
		for i in range (0, 30):
		    RHR = RHR-0.005;
		    ref.ref[ha.RHR] = RHR;
		    r.put(ref)
		    LHR = LHR-0.005;
		    ref.ref[ha.LHR] = LHR;
		    r.put(ref)
		    RAR = RAR+0.005;
		    ref.ref[ha.RAR] = RAR;
		    r.put(ref)
		    LAR = LAR+0.005;
		    ref.ref[ha.LAR] = LAR;
		    r.put(ref)
		    time.sleep(0.5)
		    print "moving knees to the right, slowly", i
		    #lift right leg
		    if (i==29):
			for j in range (0, 15):
			    RHP = RHP-0.03;
			    ref.ref[ha.RHP] = RHP;
			    r.put(ref)
			    time.sleep(0.5)
			    print "lifting right leg through pitch",j    

   	    if c == 'q':
		break

        except IOError: pass

finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)


# Close the connection to the channels
r.close()
s.close()

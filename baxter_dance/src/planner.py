#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Node that computes the current plan to be executed depending on position of centroid of Kinect point cloud
"""

import argparse
import sys
import numpy as np

import rospy

import time
import math

from std_msgs.msg import (
    String,
)

from visualization_msgs.msg import (
    Marker,
)

import baxter_interface
from baxter_interface import CHECK_VERSION

centroid_x = -999
centroid_y = -999
centroid_z = -999


def centroid_callback(data):
	global centroid_x
	global centroid_y
	global centroid_z
	centroid_x = data.pose.position.x; 
	centroid_y = data.pose.position.y;
	centroid_z = data.pose.position.z;
	print centroid_x;
	print centroid_y;
	print centroid_z;

def CheckQuadrant(x,y,z):
    if x == -999 and y == -999 and z == -999:
        return 'bottom'
    if z < 0:
        return 'bottom'
    if y > 0.15:
        return 'left'
    if y < -0.15:
        return 'right'
    else:
        return 'center'

if __name__ == "__main__":
    
    plan_names = {'bottom':'b1_v2','center':'c1_v2','right':'r1','left':'l1'}
    pub = rospy.Publisher('plan', String, queue_size=5)
    rospy.Subscriber("/visualization_marker",Marker,centroid_callback)
    rospy.init_node('planner', anonymous=True)
    head = baxter_interface.Head()
    while not rospy.is_shutdown():
        quadrant = CheckQuadrant(centroid_x,centroid_y,centroid_z)
        pub.publish(plan_names[quadrant])
        if(not centroid_z == -999): angle = math.atan2(centroid_y,centroid_x)
        else: angle = 0
        head.set_pan(angle, speed=30, timeout=0)




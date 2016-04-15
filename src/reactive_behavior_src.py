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
Node that switches to velocity mode and moves away from the dancer if a certain position/velocity threshold is crossed.
"""

import argparse
import sys
import numpy as np

import rospy

import time

from dynamic_reconfigure.server import (
    Server,
)
from std_msgs.msg import (
    Empty,
)

import baxter_interface

from baxter_examples.cfg import (
    JointSpringsExampleConfig,
)
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics


b = True
kin = None
limb_side = None


en = "enabled"

class ReactiveControl(object):
    """
    Virtual Joint Springs class for torque example.

    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server

    JointSprings class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__(self, limb, reconfig_server):
        self._dyn = reconfig_server

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = baxter_interface.Limb(limb)

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")


    def _reactive_behavior(self, vel, duration): #moves the end effector along vel for duration sec 

        global en

        #ATTENTION:
        control_rate = rospy.Rate(self._rate)

        # create our command dict
        cmd = dict()
        
        joint_index = {'s0':0, 's1':1, 'e0':2, 'e1':3, 'w0':4, 'w1':5, 'w2':6 }
        #joint_index_right = {'right_s0':0, 'right_s1':1, 'right_e0':2, 'right_e1':3, 'right_w0':4, 'right_w1':5, 'right_w2':6 }
            
        dummy_cmd = {}

        i = 0
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < duration:
            if rospy.is_shutdown():
                print("\n Aborting - ROS shutdown")
                return False
            
            # record current angles/velocities
            cur_pos = self._limb.joint_angles()
            #print cur_pos
            #cur_vel = self._limb.joint_velocities()

            jaco = kin.jacobian(cur_pos)
            #print jaco

            dummy_vel = np.dot(np.linalg.pinv(np.array(jaco)),np.array(vel).transpose())
       
            #print self._limb.joint_names()
            for joint in self._limb.joint_names():
                dummy_cmd[joint] = dummy_vel[joint_index[joint[len(joint)-2:len(joint)]]]
                if dummy_cmd[joint] > 0.7: 
                    print dummy_cmd[joint] 
                    dummy_cmd[joint] = 0.7
                    
                if dummy_cmd[joint] < -0.7:
                    print dummy_cmd[joint] 
                    dummy_cmd[joint] = -0.7
                    

            if(en == "enabled"): self._limb.set_joint_velocities(dummy_cmd)
            #print dummy_cmd
            control_rate.sleep()

        for joint in self._limb.joint_names():
            dummy_cmd[joint] = 0

        if(en == "enabled"): self._limb.set_joint_velocities(dummy_cmd)
    
    def reactive_behavior_server():
        rospy.init_node('reactive_behavior_server')
        s = rospy.Service('reactive_behavior',ReactiveBehavior ,ReactiveControl.reactive_behavior)
        print "reactive behavior ready."
        rospy.spin()
    
    

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_neutral()

    def move_to_joint_positions(self, positions):
        """
        Moves the limb to neutral location.
        """
        self._limb.move_to_joint_positions(positions)


    def clean_shutdown(self):
        """_rate
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()


def main():
    global limb_side
    global kin

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', dest='limb', required=True, choices=['left', 'right'],
        help='limb on which to attach joint springs'
    )
    parser.add_argument(
        '-f', '--file', dest='file', required=False,
        help='file from which to read trajectory to be played back with torque control'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    limb_side = args.limb
    #filename = args.file
    #print filename
    print("Initializing node... ")
    rospy.init_node("reactive_behavior_%s" % (limb_side,))
    dynamic_cfg_srv = Server(JointSpringsExampleConfig,
                             lambda config, level: config)
    
    #Initialize object from pyKDL library
    kin = baxter_kinematics(args.limb)

    rc = ReactiveControl(args.limb, dynamic_cfg_srv)
    #print keys
    #print lines
    # register shutdown callback
    rospy.on_shutdown(rc.clean_shutdown)
    rc.move_to_neutral()
    rospy.spin()
    #rc._reactive_behavior([0,0.1,0,0,0,0],10)


if __name__ == "__main__":
    main()


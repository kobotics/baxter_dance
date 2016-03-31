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
Baxter RSDK Joint Torque Example: joint springs
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


def try_float(x):
    try:
        return float(x)
    except ValueError:
        return None


def clean_line(line, names):
    """
    Cleans a single line of recorded joint positions

    @param line: the line described in a list to process
    @param names: joint name keys
    """
    #convert the line of strings to a float or None
    line = [try_float(x) for x in line.rstrip().split(',')]
    #zip the values with the joint names
    combined = zip(names[1:], line[1:])
    #take out any tuples that have a none value
    cleaned = [x for x in combined if x[1] is not None]
    #convert it to a dictionary with only valid commands
    command = dict(cleaned)
    left_command = dict((key, command[key]) for key in command.keys()
                        if key[:-2] == 'left_')
    right_command = dict((key, command[key]) for key in command.keys()
                         if key[:-2] == 'right_')
    return (command, left_command, right_command, line)



def map_file(filename, loops=1):
    """
    Loops through csv file

    @param filename: the file to play
    @param loops: number of times to loop
                  values < 0 mean 'infinite'

    Does not loop indefinitely, but only until the file is read
    and processed. Reads each line, split up in columns and
    formats each line into a controller command in the form of
    name/value pairs. Names come from the column headers
    first column is the time stamp
    """
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    #rate = rospy.Rate(1000)

    if grip_left.error():
        grip_left.reset()
    if grip_right.error():
        grip_right.reset()
    if (not grip_left.calibrated() and
        grip_left.type() != 'custom'):
        grip_left.calibrate()
    if (not grip_right.calibrated() and
        grip_right.type() != 'custom'):
        grip_right.calibrate()

    print("Playing back: %s" % (filename,))
    with open(filename, 'r') as f:
        lines = f.readlines()
    keys = lines[0].rstrip().split(',')

    return (keys,lines)


class JointSprings(object):
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

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()

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

    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._springs[joint] = self._dyn.config[joint[-2:] +
                                                    '_spring_stiffness']
            self._damping[joint] = self._dyn.config[joint[-2:] +
                                                    '_damping_coefficient']

    def _run_trajectory(self, filename): #runs a trajectory stored in file

        global en

        #ATTENTION:
        control_rate = rospy.Rate(self._rate)

        mode = "control"
        #mode = "reaction"
        #mode = "idle"

        keys, lines = map_file(filename)

        # get latest spring constants
        self._update_parameters()

        # disable cuff interaction
        #self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        #print cur_pos
        cur_vel = self._limb.joint_velocities()

        jaco = kin.jacobian(cur_pos)
        
        target_angles = self._start_angles

        torques = [0 for x in range(7)]
        joint_index = {'s0':0, 's1':1, 'e0':2, 'e1':3, 'w0':4, 'w1':5, 'w2':6 }
        #joint_index_right = {'right_s0':0, 'right_s1':1, 'right_e0':2, 'right_e1':3, 'right_w0':4, 'right_w1':5, 'right_w2':6 }
        
        dummy_force = np.array([0,2.5,0,0,0,0])
        dummy_torques = np.dot(np.array(jaco).transpose(),dummy_force.transpose())
            
        dummy_cmd = {}


        loops = 1
        l = 0
        # If specified, repeat the file playback 'loops' number of times
        while l < 1:
            i = 0
            l += 1
            print("Moving to start position...")

            _cmd, lcmd_start, rcmd_start, _raw = clean_line(lines[1], keys)
            #left.move_to_joint_positions(lcmd_start)
            self.move_to_joint_positions(rcmd_start)
            start_time = rospy.get_time()
            for values in lines[1:]:
                #print values
                i += 1
                loopstr = str(loops) if loops > 0 else "forever"
                #sys.stdout.write("\r Record %d of %d, loop %d of %s" %
                #                 (i, len(lines) - 1, l, loopstr))
                #sys.stdout.flush()

                cmd, lcmd, rcmd, values = clean_line(values, keys)
                
                #command this set of commands until the next frame
                while (rospy.get_time() - start_time) < values[0]:
                    if rospy.is_shutdown():
                        print("\n Aborting - ROS shutdown")
                        return False
                    
                    # record current angles/velocities
                    cur_pos = self._limb.joint_angles()
                    #print cur_pos
                    cur_vel = self._limb.joint_velocities()

                    if len(lcmd):
                        pass
                        #left.set_joint_positions(lcmd)
                    if len(rcmd):
                        # calculate current forces
                        for joint in target_angles.keys():
                            # spring portion
                            #ATTENTION
                            stiffness = self._springs[joint]
                            damping = self._damping[joint]*15
                            #print str(joint) + ' ' + str(stiffness)
                            
                            #messy stiffness adjustment
                            #if (joint == 'left_e1'): stiffness = 1*self._springs[joint]
                            #elif (joint == 'left_s1'): stiffness = 4*self._springs[joint]

                            if(mode == "control"):
                                
                                #get the next target angles

                                target_angles[joint] = rcmd[joint]
                                print joint , rcmd[joint]
                                
                                print joint , cur_pos[joint]
                                cmd[joint] = stiffness * (target_angles[joint] -
                                                                   cur_pos[joint])
                                # damping portion
                                cmd[joint] -= damping * cur_vel[joint]
                                
                                #torques[joint_index[joint[len(joint)-2:len(joint)]]] = cmd[joint]        
                                #if cmd[joint] > 0.2: cmd[joint] = 0.2
                                #if cmd[joint] < -0.2: cmd[joint] = -0.2

                            if(mode == "reaction"):
                                dummy_cmd[joint] = dummy_torques[joint_index[joint[len(joint)-2:len(joint)]]]

                        #right.set_joint_positions(rcmd)
                        if(mode == "control"):
                            if(en == "enabled"): self._limb.set_joint_torques(cmd)

                        if(mode == "reaction"):
                            if(en == "enabled"): self._limb.set_joint_torques(dummy_cmd)
                        
                    control_rate.sleep()
                        #time.sleep(0.1)
                    #gripper stuff (not needed)
                    # if ('left_gripper' in cmd and
                    #     grip_left.type() != 'custom'):
                    #     grip_left.command_position(cmd['left_gripper'])
                    # if ('right_gripper' in cmd and
                    #     grip_right.type() != 'custom'):
                    #     grip_right.command_position(cmd['right_gripper'])


        # #test case: oscillating elbow
        # j = "right_e1"
        # #print target_angles[j]
        # upper_limit = 1
        # lower_limit = 0.5
        # if(target_angles[j] <= upper_limit  and target_angles[j] >= lower_limit ):
        #     if(b): target_angles[j] += 0.0001
        #     else: target_angles[j] -= 0.0001
        # else:
        #     b = not b
        #     if target_angles[j] > upper_limit : target_angles[j] = upper_limit 
        #     elif target_angles[j] < lower_limit : target_angles[j] = lower_limit 
        # #end of test case 


        # print 'cmd'
        # print cmd
        print 'dummy_cmd'
        print dummy_cmd
        # print 'dummy_torques'
        # print dummy_torques
        # print ""
        # command new joint torques
        #print cmd
        #print torques
        force = np.dot(jaco,torques)
        #print force

    def _run_trajectory_simple(self): #runs a simple trajectory of one joint

        #ATTENTION:
        control_rate = rospy.Rate(self._rate)

        mode = "control"
        #mode = "reaction"
        #mode = "idle"
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """

        global b
        global en

        # get latest spring constants
        self._update_parameters()

        # disable cuff interaction
        #self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        #print cur_pos
        cur_vel = self._limb.joint_velocities()
        
        target_angles = self._start_angles

        torques = [0 for x in range(7)]
        joint_index = {'s0':0, 's1':1, 'e0':2, 'e1':3, 'w0':4, 'w1':5, 'w2':6 }
        #joint_index_right = {'right_s0':0, 'right_s1':1, 'right_e0':2, 'right_e1':3, 'right_w0':4, 'right_w1':5, 'right_w2':6 }
        

        #test case: oscillating elbow
        j = "right_e1"
        #print target_angles[j]
        upper_limit = 1
        lower_limit = 0.5
        #end of test case 


        while not rospy.is_shutdown():

            # record current angles/velocities
            cur_pos = self._limb.joint_angles()
            #print cur_pos
            cur_vel = self._limb.joint_velocities()


            #test case: osciallting elbow
            if(target_angles[j] <= upper_limit  and target_angles[j] >= lower_limit ):
                if(b): target_angles[j] += 0.0001
                else: target_angles[j] -= 0.0001
            else:
                b = not b
                if target_angles[j] > upper_limit : target_angles[j] = upper_limit 
                elif target_angles[j] < lower_limit : target_angles[j] = lower_limit 
            #end of test case: osciallting elbow

            for joint in target_angles.keys():
                #ATTENTION
                stiffness = self._springs[joint]
                #print str(joint) + ' ' + str(stiffness)
                
                #messy stiffness adjustment
                #if (joint == 'left_e1'): stiffness = 1*self._springs[joint]
                #elif (joint == 'left_s1'): stiffness = 4*self._springs[joint]

                if(mode == "control"):

                    #spring portion
                    cmd[joint] = stiffness * (target_angles[joint] -
                                                       cur_pos[joint])
                    # damping portion
                    cmd[joint] -= self._damping[joint] * cur_vel[joint]
                    
                    #torques[joint_index[joint[len(joint)-2:len(joint)]]] = cmd[joint]        
                    #if cmd[joint] > 0.2: cmd[joint] = 0.2
                    #if cmd[joint] < -0.2: cmd[joint] = -0.2

                if(mode == "reaction"):
                    dummy_cmd[joint] = dummy_torques[joint_index[joint[len(joint)-2:len(joint)]]]

            if(mode == "control"):
                if(en == "enabled"): self._limb.set_joint_torques(cmd)

            if(mode == "reaction"):
                if(en == "enabled"): self._limb.set_joint_torques(dummy_cmd)

            control_rate.sleep()

    
    def _apply_dummy_force(self, dummy_force, duration): #runs a trajectory stored in file

        global kin
        global en

        #ATTENTION:
        control_rate = rospy.Rate(self._rate*20)

        # disable cuff interaction
        #self._pub_cuff_disable.publish()

        # create our command dict
        #cmd = dict()

        torques = [0 for x in range(7)]
        joint_index = {'s0':0, 's1':1, 'e0':2, 'e1':3, 'w0':4, 'w1':5, 'w2':6 }
        #joint_index_right = {'right_s0':0, 'right_s1':1, 'right_e0':2, 'right_e1':3, 'right_w0':4, 'right_w1':5, 'right_w2':6 }
            
        dummy_cmd = {}

        start_time = time.time()
        while(time.time() - start_time < duration):
            # record current angles/velocities
            cur_pos = self._limb.joint_angles()
            #print cur_pos
            cur_vel = self._limb.joint_velocities()
            #print cur_pos
            jaco = kin.jacobian(cur_pos)
            print jaco
            dummy_torques = np.dot(np.array(jaco).transpose(),dummy_force.transpose())

            for joint in self._limb.joint_names():
                dummy_cmd[joint] = dummy_torques[joint_index[joint[len(joint)-2:len(joint)]]]

            if(en == "enabled"): self._limb.set_joint_torques(dummy_cmd)            
            control_rate.sleep()
        
        #print 'dummy_cmd'
        #print dummy_cmd
        # print 'dummy_torques'
        # print dummy_torques
        # print ""
        # command new joint torques
        #print cmd
        #print torques
        #force = np.dot(jaco,torques)
        #print force

    


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


    def attach_springs(self):
        """
        Switches to joint torque mode and attached joint springs to current
        joint positions.
        """
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()
        
        #ATTENTION
        #self._start_angles['left_e1'] = 0.2
        
        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        loops = 1
        l = 0

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            l += 1
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break
            #self._update_forces()

            #self._run_trajectory_simple()
            #self._run_trajectory('trajectories/video2')
            if l <= loops: 
                self._apply_dummy_force(np.array([0,0,-2.5,0,0,0]),20)
            print "looping"
            control_rate.sleep()

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
    """RSDK Joint Torque Example: Joint Springs

    Moves the specified limb to a neutral location and enters
    torque control mode, attaching virtual springs (Hooke's Law)
    to each joint maintaining the start position.

    Run this example on the specified limb and interact by
    grabbing, pushing, and rotating each joint to feel the torques
    applied that represent the virtual springs attached.
    You can adjust the spring constant and damping coefficient
    for each joint using dynamic_reconfigure.
    """
    global kin
    global limb_side

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
    rospy.init_node("rsdk_joint_torque_springs_%s" % (limb_side,))
    dynamic_cfg_srv = Server(JointSpringsExampleConfig,
                             lambda config, level: config)
    
    #Initialize object from pyKDL library
    kin = baxter_kinematics(args.limb)

    js = JointSprings(args.limb, dynamic_cfg_srv)
    #print keys
    #print lines
    # register shutdown callback
    rospy.on_shutdown(js.clean_shutdown)
    js.move_to_neutral()
    js.attach_springs()


if __name__ == "__main__":
    main()

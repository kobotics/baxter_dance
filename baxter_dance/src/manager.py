#Manager node which switches between three possible robot states and enables the corresponding nodes:
#1) Executing trajectory
#2) Reactive behavior upon (soft) collision
#3) Thinking about next step while wobbling

import argparse
import sys
import numpy as np

import rospy

import time

from dynamic_reconfigure.server import (
    Server,
)
from std_msgs.msg import (
    String,
)

import baxter_interface

from baxter_examples.cfg import (
    JointSpringsExampleConfig,
)
from baxter_interface import CHECK_VERSION

class DanceManager(object):
    """
    Manager tracking robot state which can take on three values: 'executing' - 'reactive' - 'thinking'
    """
    def __init__(self):
        #self._dyn = reconfig_server

        # control parameters
        self._rate = 10.0  # Hz
        
        # create our limb instances
        self._limb_left = baxter_interface.Limb('left')
        self._limb_right = baxter_interface.Limb('right')

        # cuff control (maybe later)
        # create cuff disable publisher
        # cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        # self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        self._limb_left.move_to_neutral()
        self._limb_right.move_to_neutral()

    def clean_shutdown(self):
        """_rate
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb_left.exit_control_mode()
        self._limb_right.exit_control_mode()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def get_state(self):

        #get positions and velocities
    	cur_pos_l = self._limb_left.joint_angles()
        cur_vel_l = self._limb_left.joint_velocities()

       	cur_pos_r = self._limb_right.joint_angles()
        cur_vel_r = self._limb_right.joint_velocities()

        #check for collision against Kinect information

        v_threshold = 0.5
        high_v_l = filter(lambda x: x > v_threshold, cur_vel_r.values())
        high_v_r = filter(lambda x: x > v_threshold, cur_vel_r.values())
        if high_v_l != [] or high_v_l != []: #need to add perception data
        	return 'reactive'

if __name__ == "__main__":
    
    pub = rospy.Publisher('state', String, queue_size=10)
    print("Initializing node... ")
    rospy.init_node('manager', anonymous=True)
	
    manager = DanceManager()

    #clean shutdown methods
    rospy.on_shutdown(manager.clean_shutdown)
    manager.move_to_neutral()
    manager_rate = rospy.Rate(manager._rate)

    while not rospy.is_shutdown():
	    state = manager.get_state()
	    pub.publish(state)
	    manager_rate.sleep()

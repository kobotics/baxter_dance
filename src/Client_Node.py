import sys
import rospy

try:
    add_two_ints = rospy.ServiceProxy('reactive_behavior',ReactiveBehavior)
except rospy.ServiceException, e:
    print "Service call failed: %s"%e
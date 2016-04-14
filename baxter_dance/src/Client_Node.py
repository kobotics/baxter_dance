import sys
import rospy
def reactive_behavior_client(v, d):
    rospy.wait_for_service('reactive_behavior')
    try:
        reactive_behavior = rospy.ServiceProxy('reactive_behavior',ReactiveBehavior)
        resp1 = reactive_behavior(v,d)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print reactive_behavior_client([0.1,0,0],1)

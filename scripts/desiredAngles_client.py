#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from beginner_tutorials.srv import *

def set_desired_angles_client(a1, a2, a3, a4, a5, a6):
    rospy.wait_for_service('set_desired_angles')
    try:
        desired_Angles = rospy.ServiceProxy('set_desired_angles', desiredAngles)
        state = desired_Angles(a1, a2, a3, a4, a5, a6)
        return state.done
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s a1 a2 a3 a4 a5 a6"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 7:
        a1 = int(sys.argv[1])
        a2 = int(sys.argv[2])
	a3 = int(sys.argv[3])
        a4 = int(sys.argv[4])
	a5 = int(sys.argv[5])
        a6 = int(sys.argv[6])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting [%f %f %f %f %f %f]"%(a1, a2, a3, a4, a5, a6))
    result = set_desired_angles_client(a1, a2, a3, a4, a5, a6)
    print("Desired Axis Angles reached: " + str(result))
    #print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))

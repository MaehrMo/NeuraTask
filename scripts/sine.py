#!/usr/bin/env python
from __future__ import print_function
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import math
import actionlib
from std_msgs.msg import Float32
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

client = None

def move_sine():
    rate = rospy.Rate(10) # 10hz
    var = 0.0
    while not rospy.is_shutdown():
	sine = 0.5 * math.pi * math.sin(var)
        rospy.loginfo(sine)
        rate.sleep()
	var += 0.01
	if var > math.pi:
	    var = -math.pi

    	AxisAngles = [sine]*6
    	g = FollowJointTrajectoryGoal()
    	g.trajectory = JointTrajectory()
    	g.trajectory.joint_names = JOINT_NAMES
    
        d = 2.0
    	g.trajectory.points = []
        g.trajectory.points.append(JointTrajectoryPoint(positions=AxisAngles, velocities=[0]*6, time_from_start=rospy.Duration(d)))
        d += 2

        #rospy.loginfo(AxisAngles)
        client.send_goal(g)
        #try:
	    #rate.sleep()
        #client.wait_for_result()


def main():
    global client
    try:
        rospy.init_node("neura_sine_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Connected to server")
	move_sine()
	#rospy.spin()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()


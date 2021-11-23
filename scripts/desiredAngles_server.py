#!/usr/bin/env python

from __future__ import print_function
import time
import roslib; roslib.load_manifest('ur_driver')
from beginner_tutorials.srv import desiredAngles,desiredAnglesResponse
import rospy
import math
import actionlib
from std_msgs.msg import Float32
from control_msgs.msg import *
from trajectory_msgs.msg import *

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']							#joint names of ur5

client = None

def handle_set_desired_angles(req):
    client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)	#hook up to simulation server
    print("Waiting for server...")
    client.wait_for_server()												#wait until connected to simulation server
    print("Connected to server")
    #print("Returning [%f %f %f %f %f %f]"%(req.a1, req.a2, req.a3, req.a4, req.a5, req.a6))
    duration = 2.0    													#need a time_from_start
    AxisAngles = [req.a1, req.a2, req.a3, req.a4, req.a5, req.a6]
    goal = FollowJointTrajectoryGoal()											#get empty JointTrajectoryGoal object
    goal.trajectory = JointTrajectory()											#get empty JointTrajectory object
    goal.trajectory.joint_names = JOINT_NAMES										#fill in JOINT_NAMES of ur5
    goal.trajectory.points = []
    
    goal.trajectory.points.append(											#append the AxisAngles from service call as new Goal
        	JointTrajectoryPoint(positions=AxisAngles, velocities=[0]*6, time_from_start=rospy.Duration(duration)))
    #d += 2
    rospy.loginfo(AxisAngles)												#log info of Axis Angles
    client.send_goal(goal)												#send goal to move
    return desiredAnglesResponse(client.wait_for_result()) 								#return true if reached end pos

def main():
    rospy.init_node("neura_desired_joint_angles", anonymous=True, disable_signals=True)					#init ROS node  
    desiredAnglesService = rospy.Service('neuraTask/set_desired_angles', desiredAngles, handle_set_desired_angles)	#setup rospy service
    print("Ready to recieve angles")
    rospy.spin()													#keep program running


if __name__ == "__main__":
    main()
    

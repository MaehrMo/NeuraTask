#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
from neura_task.srv import angularOrder,angularOrderResponse
from neura_task.srv import linearOrder,linearOrderResponse

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterface(object):

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)								#initialize `moveit_commander`_ and a `rospy`_ node

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).
        ## This interface can be used to plan and execute motions:

        group_name = "manipulator" 										#for ur5 "manipulator" is the right name
        move_group = moveit_commander.MoveGroupCommander(group_name)						#link move_group with the "manipulator" name of the ur5 robot

	print("Current Position")
	print(move_group.get_current_pose().pose)								#get the current position values to reference values for goals
        
	# We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        
        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        
        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        
        # Sometimes for debugging it is useful to print the entire state of the robot
        #print("Printing robot state")
        #print(robot.get_current_state())
        #print("")

        # Save variables in Object
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    #def go_to_joint_state(self):										#theoretically Task1
        
        #move_group = self.move_group										#get move_group from object

        #joint_goal = move_group.get_current_joint_values()
        #joint_goal[0] = 0											#A1
        #joint_goal[1] = -tau / 8										#A2
        #joint_goal[2] = 0											#A3
        #joint_goal[3] = -tau / 4										#A4
        #joint_goal[4] = 0											#A5
        #joint_goal[5] = tau / 6  # 1/6 of a turn								#A6

        #move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        #move_group.stop()

        # For testing:
        #current_joints = move_group.get_current_joint_values()
        #return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose):
        
        move_group = self.move_group										#get move_group from object

        ## Planning to a Pose Goal
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()									#get empty Pose object
	pose_goal.orientation.x = pose.orientation.x								#fill in X, Y, Z, rX, rY, rZ and w from .srv
	pose_goal.orientation.y = pose.orientation.y
	pose_goal.orientation.z = pose.orientation.z
        pose_goal.orientation.w = pose.orientation.w
        pose_goal.position.x = pose.position.x
        pose_goal.position.y = pose.position.y
        pose_goal.position.z = pose.position.z

        move_group.set_pose_target(pose_goal)									#set the goal pose

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)										#move to goal

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
	all_close(pose_goal, current_pose, 0.01)
        return plan
	
    def go_to_point_goal(self, point):

        move_group = self.move_group										#get move_group from object

        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()									#get empty pose object
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = point.x										#fill in X, Y, Z from .srv
        pose_goal.position.y = point.y
        pose_goal.position.z = point.z

        move_group.set_pose_target(pose_goal)									#set as new goal to reach

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)									#move to new goal and wait until finished

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
	all_close(pose_goal, current_pose, 0.01)
        return plan

    def plan_cartesian_path(self, pose):

        move_group = self.move_group										#get move_group from object
							
	waypoints = []												#empty waypoint array

        wpose = move_group.get_current_pose().pose								#get current pose of robot
	wpose.orientation.x = pose.orientation.x								#fill in X, Y, Z, rX, rY, rZ and w from .srv
	wpose.orientation.y = pose.orientation.y
	wpose.orientation.z = pose.orientation.z
        wpose.orientation.w = pose.orientation.w
	wpose.position.x = pose.position.x
        wpose.position.y = pose.position.y  
	wpose.position.z = pose.position.z  
        waypoints.append(copy.deepcopy(wpose))									#append as many points as needed

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient.

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan):
        
        move_group = self.move_group										#get move_group from object

        ## Use execute if you would like the robot to follow the computed plan:
        return move_group.execute(plan, wait=True)


def handle_linear_movement(input):
    
    linearCommander = MoveGroupPythonInterface()											#initialize a MoveIt Interface

    moveit_commander.move_group.MoveGroupCommander.set_max_velocity_scaling_factor(linearCommander.move_group, input.lin_vel)		#set linear velocity function
    moveit_commander.move_group.MoveGroupCommander.set_max_acceleration_scaling_factor(linearCommander.move_group, input.lin_acc)	#set linear acceleration function
    
    done1 = linearCommander.go_to_pose_goal(input.pose1)										#PTP-Movement to pose1
    print("Reached Pose1: " + str(done1))
    
    cartesian_plan, fraction = linearCommander.plan_cartesian_path(input.pose2)								#plan LIN-Movement to point2

    done2 = linearCommander.execute_plan(cartesian_plan)										#execute LIN-Movement to point2
    print("Reached Pose2: " + str(done2))

    return (done1 and done2)

def handle_angular_movement(input):

    angularCommander = MoveGroupPythonInterface()											#initialize a MoveIt Interface
    moveit_commander.move_group.MoveGroupCommander.set_max_velocity_scaling_factor(angularCommander.move_group, input.ang_vel)		#set angular velocity function	
    moveit_commander.move_group.MoveGroupCommander.set_max_acceleration_scaling_factor(angularCommander.move_group, input.ang_acc)	#set angular acceleration function	
    
    done1 = angularCommander.go_to_point_goal(input.point1)										#PTP-Movement to point1 
    print("Reached Point1: " + str(done1))

    done2 = angularCommander.go_to_point_goal(input.point2)										#PTP-Movement to point2 
    print("Reached Point1: " + str(done2))

    return (done1 and done2)

def main():
    try:
        
	rospy.init_node("neura_move_server", anonymous=True, disable_signals=True)							#Init Server with rosservice calls in Terminal
  	angularMove = rospy.Service('neuraTask/move_server/angularMovement', angularOrder, handle_angular_movement)			#Rosservice with PTP-Movements
  	linearMove = rospy.Service('neuraTask/move_server/linearMovement', linearOrder, handle_linear_movement)				#Rosservice with LIN-Movement between pose1 and pose2
        print("Set up services")
	rospy.spin()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()



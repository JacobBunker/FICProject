#!/usr/bin/env python
#3.6

import argparse
import random
import struct
import sys
import Queue

print(sys.version)

import rospy

import subprocess

import baxter_interface

from baxter_interface import CHECK_VERSION

import ikpy
import numpy as np
from ikpy import plot_utils

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


from sensor_msgs.msg import JointState

#compares each element in order in arrays a and b and returns the largest numerical difference found
def getDiff(a,b): 
	print(a)
	print(b)
	j = -1
	m = -1
	i = 0
	while(i < len(a)):
		c = np.absolute(a[i] - b[i])
		if(c > m):
			j = i
			m = c
		i += 1
	print(j)
	return m

#creates a dict out of a key array a and a value array b
def convertToDict(a,b):
	p = {}
	i = 0
	while(i < len(a)):
		p[a[i]] = b[i]
		i += 1
	return p

#rearange dictionary elements into a given order
def properOrder(angles,order):
	starting = []
	i = 0
	while(i < len(order)):
		if(order[i] in angles):
			starting.append(angles[order[i]])
		else:
			starting.append(0.)
		i += 1
	return np.asarray(starting)


def convertToStampedPose(hdr, sP):
	return PoseStamped(header=hdr,pose=
		Pose(
			position=Point(
				x=sP["position"][0],
				y=sP["position"][1],
				z=sP["position"][2]),
			orientation=Quaternion(
				x=sP["orientation"][0],
				y=sP["orientation"][1],
				z=sP["orientation"][2],
				w=sP["orientation"][3])
		))

jointStates = Queue.LifoQueue(maxsize=8)

def callback(data):
	if(jointStates.full()):
		jointStates.get()
	jointStates.put(data)
	#print(data)

class Wobbler(object):
	def __init__(self):
		self._done = False
		self._head = baxter_interface.Head()
		self._left_arm = baxter_interface.limb.Limb('left')
		self._right_arm = baxter_interface.limb.Limb('right')
		print(self._left_arm.joint_names())
		print(self._left_arm.joint_angles())
		print(self._left_arm.joint_velocities())
		print(self._left_arm.joint_efforts())
		print(self._left_arm.endpoint_pose())
		print(self._left_arm.endpoint_velocity())
		print(self._left_arm.endpoint_effort())

		self._left_arm_chain = Chain(name='left_arm', 
		active_links_mask=[False,False,True,True,True,True,True,True,True,False],
		links=[
		    OriginLink(),
		    URDFLink(
		      name="left_torso_arm_mount",
		      translation_vector=[0.024645, 0.219645, 0.118588],
		      orientation=[0, 0, 0.7854],
		      rotation=[0, 0, 1],
		    ),
		    URDFLink(
		      name="left_s0",
		      translation_vector=[0.055695, 0, 0.011038],
		      orientation=[0, 0, 0],
		      rotation=[0, 0, 1],
		      bounds=(-1.70167993878, 1.70167993878)
		    ),
		    URDFLink(
		      name="left_s1",
		      translation_vector=[0.069, 0, 0.27035],
		      orientation=[-1.57079632679, 0, 0],
		      rotation=[0, 0, 1],
		      bounds=(-2.147, 1.047)
		    ),
		    URDFLink(
		      name="left_e0",
		      translation_vector=[0.102, 0, 0],
		      orientation=[1.57079632679, 0, 1.57079632679],
		      rotation=[0, 0, 1],
		      bounds=(-3.05417993878, 3.05417993878)
		    ),
		    URDFLink(
		      name="left_e1",
		      translation_vector=[0.069, 0, 0.26242],
		      orientation=[-1.57079632679, -1.57079632679, 0],
		      rotation=[0, 0, 1],
		      bounds=(-0.05, 2.618)
		    ),
		    URDFLink(
		      name="left_w0",
		      translation_vector=[0.10359, 0, 0],
		      orientation=[1.57079632679, 0, 1.57079632679],
		      rotation=[0, 0, 1],
		      bounds=(-3.059, 3.059)
		    ),
		    URDFLink(
		      name="left_w1",
		      translation_vector=[0.01, 0, 0.2707],
		      orientation=[-1.57079632679, -1.57079632679, 0],
		      rotation=[0, 0, 1],
		      bounds=(-1.57079632679, 2.094)
		    ),
		    URDFLink(
		      name="left_w2",
		      translation_vector=[0.115975, 0, 0],
		      orientation=[1.57079632679, 0, 1.57079632679],
		      rotation=[0, 0, 1],
		      bounds=(-3.059, 3.059)
		    ),
		    URDFLink(
		      name="left_hand",
		      translation_vector=[0, 0, 0.11355],
		      orientation=[0, 0, 0],
		      rotation=[0, 0, 1],
		    )
		])
		#self._left_arm_chain = ikpy.chain.Chain.from_urdf_file("/home/jbunker/ros_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")


	
		# verify robot is enabled
		print("Getting robot state... ")
		self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot... ")
		self._rs.enable()
		print("Running. Ctrl-c to quit")

	def clean_shutdown(self):
		"""
		Exits example cleanly by moving head to neutral position and
		maintaining start state
		"""
		print("\nExiting example...")
		if self._done:
		    self.set_neutral()
		if not self._init_state and self._rs.state().enabled:
		    print("Disabling robot...")
		    self._rs.disable()

	def set_neutral(self):
		"""
		Sets the head back into a neutral pose
		"""
		self._head.set_pan(0.0)

	def testJoint(self):
		self.set_neutral()
		print(self._left_arm.joint_names())
		target = (self._right_arm.joint_names())[0]
		print("targeting joint {0}".format(target))
		print("starting angle {0}".format(self._right_arm.joint_angle(target)))
		print("starting velocity {0}".format(self._right_arm.joint_velocity(target)))
		print("starting effort {0}".format(self._right_arm.joint_effort(target)))
		
		command_rate = rospy.Rate(1)
		control_rate = rospy.Rate(50)
		
		commandDictLeft = {}
		for n in self._left_arm.joint_names():
			commandDictLeft[n] = 0.
		
		commandDictRight = {}
		for n in self._right_arm.joint_names():
			commandDictRight[n] = 0.
		
		
		start = rospy.get_time()
		while not rospy.is_shutdown() and (rospy.get_time() - start < 0.1):
			self._left_arm.set_joint_positions(commandDictLeft)
			self._right_arm.set_joint_positions(commandDictRight)
			control_rate.sleep()
		
		print("zeroing ended, recording starting waypoint...")
		
		i = 0
		while(i < 3):
			state = jointStates.get()
			print(i)
			i += 1

		originP = [0.064027, 0.259027, 0.08]

		target = np.asarray([[1, 0, 0, .7],
                          	     [0, 1, 0, .0],
                          	     [0, 0, 1, .5],
                          	     [0, 0, 0, 1]])

		angles = self._left_arm.joint_angles()
		order = ["origin", "left_torso_arm_mount", "left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2", "left_hand"]
		angles = properOrder(self._left_arm.joint_angles(),order)

		print(angles)
		print(target)
		print(self._left_arm_chain)

		solution = ikpy.inverse_kinematics.inverse_kinematic_optimization(chain=self._left_arm_chain, target_frame=target, starting_nodes_angles=angles)
		print(solution)

		inp = convertToDict(order,solution)
		
		i = 0
		while(getDiff(angles,solution) > 0.01 and i < 1000):
			print(getDiff(angles,solution))
			inp = convertToDict(order,solution)
			print(inp)
			self._left_arm.set_joint_positions(convertToDict(order,solution))
			#self._left_arm.set_joint_positions(commandDictLeft)
			control_rate.sleep()
			angles = properOrder(self._left_arm.joint_angles(),order)
			#print(angles)
			i += 1
		print(self._left_arm_chain)

		ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

		self._left_arm_chain.plot(ikpy.inverse_kinematics.inverse_kinematic_optimization(chain=self._left_arm_chain, target_frame=target, starting_nodes_angles=angles), ax)
		matplotlib.pyplot.show()
			

		#t_angles = self.left_arm_chain.inverse_kinematics(target=target_frame,initial_position=self._left_arm.joint_positions()))
	
		self._done = True
		rospy.signal_shutdown("Example finished.")

def main():
	"""RSDK Head Example: Wobbler

	Nods the head and pans side-to-side towards random angles.
	Demonstrates the use of the baxter_interface.Head class.
	"""
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
		                             description=main.__doc__)
	parser.parse_args(rospy.myargv()[1:])

	print("Initializing node... ")
	rospy.init_node("rsdk_dabber")#, log_level=rospy.DEBUG)
	rospy.Subscriber("/robot/joint_states", JointState, callback)

	wobbler = Wobbler()
	rospy.on_shutdown(wobbler.clean_shutdown)
	print("Wobbling... ")
	wobbler.testJoint()
	print("Done.")

if __name__ == '__main__':
	main()

#!/usr/bin/env python

import argparse
import random
import struct
import sys

import rospy

import baxter_interface

from baxter_interface import CHECK_VERSION

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

def makeIKReq(ns,target):
	iksvc = rospy.ServiceProxy(ns,SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	ikreq.pose_stamp.append(target)
	print(ikreq)
    	try:
        	rospy.wait_for_service(ns, 5.0)
        	resp = iksvc(ikreq)
    	except (rospy.ServiceException, rospy.ROSException), e:
        	rospy.logerr("Service call failed: %s" % (e,))
        	return 1
	# Check if result valid, and type of seed ultimately used to get solution
    	# convert rospy's string representation of uint8[]'s to int's
    	resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    	if (resp_seeds[0] != resp.RESULT_INVALID):
        	seed_str = {
                    	ikreq.SEED_USER: 'User Provided Seed',
                    	ikreq.SEED_CURRENT: 'Current Joint Angles',
                    	ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   	}.get(resp_seeds[0], 'None')
        	#print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
        	# Format solution into Limb API-compatible dictionary
        	limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        	#print "\nIK Joint Solution:\n", limb_joints
        	#print "------------------"
        	#print "Response Message:\n", resp
		return limb_joints
    	else:
        	#print(resp_seeds)
		#print(resp)
        	print("INVALID POSE - No Valid Joint Solution Found.")
		return None

#takes two dicts, assumes same keys in a exist in b,
#gets difference between float values between same keys
def getAngleDiff(a,b):
	diff = 0.
	keyList = a.keys()
	for k in keyList:
		diff = diff + abs(a[k] - b[k])
	return diff


class Wobbler(object):

    def __init__(self):
        """
        'Wobbles' the head
        """
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
            self.set_neutral(1)
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()

    def set_neutral(self, opt):

        """
        Sets the body back into a neutral pose
        """

        self._head.set_pan(0.0)
	command_rate = rospy.Rate(1)
	control_rate = rospy.Rate(50)

	commandDictLeft = {}
	for n in self._left_arm.joint_names():
		commandDictLeft[n] = 0.
	
	commandDictRight = {}
	for n in self._right_arm.joint_names():
		commandDictRight[n] = 0.

	diff = 10.
	start = rospy.get_time()
	while not rospy.is_shutdown() and (diff > 0.02):
		self._left_arm.set_joint_positions(commandDictLeft)
		self._right_arm.set_joint_positions(commandDictRight)
		control_rate.sleep()
		diff = getAngleDiff(commandDictLeft,self._left_arm.joint_angles())
	
	if(opt > 0):
		commandDictLeft = {}
		for n in self._left_arm.joint_names():
			commandDictLeft[n] = 0.

		commandDictLeft["left_s1"] = 0.7
		commandDictLeft["left_e1"] = 0.5
	
		commandDictRight = {}
		for n in self._right_arm.joint_names():
			commandDictRight[n] = 0.

		commandDictRight["right_s1"] = 0.7
		commandDictRight["right_e1"] = 0.5

		diff = 10.
		start = rospy.get_time()
		while not rospy.is_shutdown() and (diff > 0.02):
			self._left_arm.set_joint_positions(commandDictLeft)
			self._right_arm.set_joint_positions(commandDictRight)
			control_rate.sleep()
			diff = getAngleDiff(commandDictLeft,self._left_arm.joint_angles())

    def testJoint(self):
	command_rate = rospy.Rate(1)
	control_rate = rospy.Rate(50)
	
	armSpeed = 0.1
	start = rospy.get_time()
	while not rospy.is_shutdown() and (rospy.get_time() - start < 1.):
		self._right_arm.set_joint_position_speed(armSpeed)
		self._left_arm.set_joint_position_speed(armSpeed)
		command_rate.sleep()

	self.set_neutral(0)

	print("neutral position set")

	text = ''
	while(text != 'y' and text != 'n'):
		text = raw_input("stage 1 ['y': continue, 'n': move to rest]") 
	if(text == 'n'):
		print("moving to rest position")
		self._done = True
        	rospy.signal_shutdown("Example finished.")

	commandDictRight = {}
	commandDictLeft = {}
	commandDictLeft["left_s0"] = -0.675
	commandDictLeft["left_s1"] = -0.4
	commandDictLeft["left_e0"] = 0.
	commandDictLeft["left_e1"] = 0.6
	commandDictLeft["left_w0"] = 0.
	commandDictLeft["left_w1"] = 0.9

	print("moving to obtain new position")
	start = rospy.get_time()
	targetAngle = 0 #init to start

	diff = 10.
	while not rospy.is_shutdown() and (diff > 0.02):
		self._left_arm.set_joint_positions(commandDictLeft)
		control_rate.sleep()
		diff = getAngleDiff(commandDictLeft,self._left_arm.joint_angles())

	text = ''
	while(text != 'y' and text != 'n'):
		text = raw_input("stage 2 ['y': continue, 'n': move to rest]") 
	if(text == 'n'):
		print("moving to rest position")
		self._done = True
        	rospy.signal_shutdown("Example finished.")



	commandDictRight = {}
	commandDictLeft = {}
	commandDictLeft["left_s0"] = -0.675
	commandDictLeft["left_s1"] = -0.32
	commandDictLeft["left_e0"] = 0.
	commandDictLeft["left_e1"] = 0.6
	commandDictLeft["left_w0"] = 0.
	commandDictLeft["left_w1"] = 0.9

	print("moving to obtain new position")
	start = rospy.get_time()
	targetAngle = 0 #init to start

	diff = 10.
	while not rospy.is_shutdown() and (diff > 0.02):
		self._left_arm.set_joint_positions(commandDictLeft)
		control_rate.sleep()
		diff = getAngleDiff(commandDictLeft,self._left_arm.joint_angles())

	text = ''
	while(text != 'y' and text != 'n'):
		text = raw_input("stage 3 ['y': continue, 'n': move to rest]") 
	if(text == 'n'):
		print("moving to rest position")
		self._done = True
        	rospy.signal_shutdown("Example finished.")

	commandDictRight = {}
	commandDictLeft = {}
	commandDictLeft["left_s0"] = -0.3
	commandDictLeft["left_s1"] = -0.32
	commandDictLeft["left_e0"] = 0.
	commandDictLeft["left_e1"] = 0.6
	commandDictLeft["left_w0"] = 0.
	commandDictLeft["left_w1"] = 0.9

	print("moving to obtain new position")
	start = rospy.get_time()
	targetAngle = 0 #init to start

	diff = 10.
	while not rospy.is_shutdown() and (diff > 0.02):
		self._left_arm.set_joint_positions(commandDictLeft)
		control_rate.sleep()
		diff = getAngleDiff(commandDictLeft,self._left_arm.joint_angles())

	text = ''
	while(text != 'y' and text != 'n'):
		text = raw_input("stage 4 ['y': continue, 'n': move to rest]") 
	if(text == 'n'):
		print("moving to rest position")
		self._done = True
        	rospy.signal_shutdown("Example finished.")

	commandDictRight = {}
	commandDictLeft = {}
	commandDictLeft["left_s0"] = -0.675
	commandDictLeft["left_s1"] = -0.32
	commandDictLeft["left_e0"] = 0.
	commandDictLeft["left_e1"] = 0.6
	commandDictLeft["left_w0"] = 0.
	commandDictLeft["left_w1"] = 0.9

	print("moving to obtain new position")
	start = rospy.get_time()
	targetAngle = 0 #init to start

	diff = 10.
	while not rospy.is_shutdown() and (diff > 0.02):
		self._left_arm.set_joint_positions(commandDictLeft)
		control_rate.sleep()
		diff = getAngleDiff(commandDictLeft,self._left_arm.joint_angles())

	text = ''
	while(text != 'y' and text != 'n'):
		text = raw_input("stage 5 ['y': continue, 'n': move to rest]") 
	if(text == 'n'):
		print("moving to rest position")
		self._done = True
        	rospy.signal_shutdown("Example finished.")

	commandDictRight = {}
	commandDictLeft = {}
	commandDictLeft["left_s0"] = -0.675
	commandDictLeft["left_s1"] = -0.45
	commandDictLeft["left_e0"] = 0.
	commandDictLeft["left_e1"] = 0.6
	commandDictLeft["left_w0"] = 0.
	commandDictLeft["left_w1"] = 0.9

	print("moving to obtain new position")
	start = rospy.get_time()
	targetAngle = 0 #init to start

	diff = 10.
	while not rospy.is_shutdown() and (diff > 0.02):
		self._left_arm.set_joint_positions(commandDictLeft)
		control_rate.sleep()
		diff = getAngleDiff(commandDictLeft,self._left_arm.joint_angles())

	text = ''
	while(text != 'y' and text != 'n'):
		text = raw_input("stage 6 ['y': continue, 'n': move to rest]") 
	if(text == 'n'):
		print("moving to rest position")
		self._done = True
        	rospy.signal_shutdown("Example finished.")


	text = raw_input("final stage [enter to continue]") 
	print("moving to rest position")
	self._done = True
        rospy.signal_shutdown("Example finished.")

#	text = ''
#	while(text != 'end'):
#		text = raw_input("custom stage ['string/float' or 'end']") 
#		inp = text.split('/')
#		print(inp)
#		if inp[0] in commandDictLeft:
#			commandDictLeft[inp[0]] = float(inp[1])
#		diff = 10.
#		while not rospy.is_shutdown() and (diff > 0.02):
#			self._left_arm.set_joint_positions(commandDictLeft)
#			control_rate.sleep()
#			diff = getAngleDiff(commandDictLeft,self._left_arm.joint_angles())


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

    wobbler = Wobbler()
    rospy.on_shutdown(wobbler.clean_shutdown)
    print("Wobbling... ")
    wobbler.testJoint()
    print("Done.")

if __name__ == '__main__':
    main()

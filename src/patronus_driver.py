#!/usr/bin/env python

import rospy
import numpy as np
import random
import sys
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.srv import Spawn, SetPen
from turtlesim.msg import Color, Pose

#pose values for turtle_name turtle
x = 0
y = 0
theta = 0

command = "WALK BESIDE ME"

# used to calculate force due to the other turtles
turtle_poses = dict({"angel":[0,0,0],
					 "devil":[0,0,0],
					 "turtle1":[0,0,0],
	                 "jon":[0,0,0]})

def locate(turtle_name):
	global turtle_poses
	return turtle_poses[turtle_name]

# used to set global variables for the pose of this node's turtle's position
def pose_callback(data):
	global x
	global y
	global theta
	x = data.x
	y = data.y
	theta = float(np.mod(data.theta,2*np.pi)) # store angle value within +2*pi

# used to set global variable for the pose of all turtles' positions
def t_pose_callback(data):
	topic_name=data._connection_header['topic']
	turtle_name=topic_name.replace('/',"").replace("pose","")
	global turtle_poses
	turtle_poses[turtle_name][0] = data.x
	turtle_poses[turtle_name][1] = data.y
	turtle_poses[turtle_name][2] = data.theta

def command_callback(data):
	global command
	# For whom is this command?
	if data.data.find("ANGEL")>=0 or data.data.find("DEVIL")>=0:
		who = data.data.split(" ")[0].replace(",","")
	else:
		who = "both"
	# Only act if it's for me or both of us.
	if who.lower() == rospy.myargv(argv=sys.argv)[1]:
		command = data.data.replace(who,"").replace(",","").strip()
	elif who.lower() == "both":
		command = data.data

# pose_offset is a vector [r,theta] and angle_offset is the goal angle of the patronus relative to the goal turtle
def go_to(turtle_name,pose_offset,angle_offset):

	pub = rospy.Publisher(rospy.myargv(argv=sys.argv)[1]+'/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(50) #hz

	t_pose = locate(turtle_name)
	t_angle = t_pose[2]
	if t_angle<0:
		t_angle = t_angle + 2*np.pi
	r = pose_offset[0]
	th = pose_offset[1]
	goal_pose = np.array([t_pose[0],t_pose[1]]) + r*np.array([np.cos(t_angle+th),np.sin(t_angle+th)])
	current_pose = np.array([x,y])
	direction = goal_pose-current_pose
	goal_angle = np.mod(np.arctan2(direction[1],direction[0]),2*np.pi)
	i = np.sign(goal_angle-theta)
	if abs(goal_angle-theta) > np.pi:
		i = -i
	ang = Vector3(0,0,.5*i)
	lin = Vector3(0,0,0)
	while abs(goal_angle-theta)>.01:
		stumble = Twist(lin,ang)
		pub.publish(stumble)
		rate.sleep()
	ang = Vector3(0,0,0)
	lin = Vector3(3,0,0)
	while np.linalg.norm(goal_pose-current_pose)>.1:
		current_pose = np.array([x,y])
		stumble = Twist(lin,ang)
		pub.publish(stumble)
		rate.sleep()
	goal_angle = float(np.mod(t_angle + angle_offset,2*np.pi))
	i = np.sign(goal_angle-theta)
	if abs(goal_angle-theta) > np.pi:
		i = -i
	ang = Vector3(0,0,.5*i)
	lin = Vector3(0,0,0)
	while abs(theta-goal_angle)>.1:
		stumble = Twist(lin,ang)
		pub.publish(stumble)
		rate.sleep()
	ang = Vector3(0,0,0)
	lin = Vector3(0,0,0)
	stumble = Twist(lin,ang)
	pub.publish(stumble)
	rate.sleep()

def driver(turtle_name):
	global x
	global y
	global theta
	global turtle_poses
	global command

	# get all turtles' poses
	rospy.Subscriber(turtle_name+'/pose',Pose,pose_callback)
	for other in ["turtle1","jon","angel","devil"]:
		rospy.Subscriber(other+'/pose',Pose,t_pose_callback)

	rospy.Subscriber('/hlpr_speech_commands',String,command_callback)

	pub = rospy.Publisher(turtle_name+'/cmd_vel', Twist, queue_size=10)

	rospy.init_node(turtle_name+'_driver', anonymous=True)
	rate = rospy.Rate(50) #hz

	while not rospy.is_shutdown():

		if command == "STAY WHERE YOU ARE":
			ang = Vector3(0,0,0)
			lin = Vector3(0,0,0)
			stumble = Twist(lin,ang)

		elif command == "SEND JON A MESSAGE":
			go_to('jon',[1,0],np.pi) # stand in front of jon and face him
			print("Hi Jon!!")
			command = "STAY WHERE YOU ARE"

		elif command == "COME HERE":
			if turtle_name == "angel":
				go_to('turtle1',[1,3*np.pi/2],0) # go next to turtle1 and face the same direction as her
			else:
				go_to('turtle1',[1,np.pi/2],0) # go next to turtle1 and face the same direction as her
			print("Hi Ilse!")
			command = "STAY WHERE YOU ARE"

		elif command == "WALK BESIDE ME":
			t1_pose = locate('turtle1')
			t1_angle = t1_pose[2]
			current_pose = np.array([x,y])
			if turtle_name == "angel":
				goal_pose = np.array([t1_pose[0],t1_pose[1]]) + 1*np.array([np.cos(t1_angle+3*np.pi/2),np.sin(t1_angle+3*np.pi/2)])
			else:
				goal_pose = np.array([t1_pose[0],t1_pose[1]]) + 1*np.array([np.cos(t1_angle+np.pi/2),np.sin(t1_angle+np.pi/2)])
			d = np.linalg.norm(goal_pose-current_pose)

			direction = goal_pose-current_pose
			goal_angle = float(np.mod(np.arctan2(direction[1],direction[0]),2*np.pi))
			angle_difference = goal_angle-theta
			if angle_difference > np.pi:
				angle_difference = angle_difference-2*np.pi
			elif angle_difference < -np.pi:
				angle_difference = angle_difference + 2*np.pi
			ang = Vector3(0,0,10*angle_difference)
			lin = Vector3(10*np.linalg.norm(goal_pose-current_pose),0,0) # go faster if you are farther from ilse
			stumble = Twist(lin,ang)
			pub.publish(stumble)
			rate.sleep()

# Spawn the turtle for this node
def mommy(baby_name):
	global x
	global y
	global theta

	rospy.wait_for_service('spawn')
	try:
		new_turtle = rospy.ServiceProxy('spawn',Spawn)
		if baby_name == "angel":
			resp = new_turtle(5.5,4.5,0,baby_name)
		else:
			resp = new_turtle(5.5,6.5,0,baby_name)
	except rospy.ServiceException, e:
		print "%s"%e

	rospy.wait_for_service(baby_name+'/set_pen')
	try:
		changepen = rospy.ServiceProxy(baby_name+'/set_pen',SetPen)
		changepen(0,0,0,1,255)
	except rospy.ServiceException, e:
		print "%s"%e

if __name__ == '__main__':

	baby_name = rospy.myargv(argv=sys.argv)[1]

	mommy(baby_name)
	try:
		driver(baby_name)
	except rospy.ROSInterruptException:
		pass




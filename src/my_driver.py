#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from turtlesim.srv import Kill, Spawn, SetPen
from turtlesim.msg import Pose

x = 0
y = 0
theta = 0
command = "WALK WITH ME"

def pose_callback(data):
	global x
	global y
	global theta
	x = data.x
	y = data.y
	theta = float(np.mod(data.theta,2*np.pi)) # bring angle value to within +2*pi

def command_callback(data):
	global command
	command = data.data

def check_for_walls(turtle_name):

	pub = rospy.Publisher(turtle_name+'/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(100) # hz
	i=0

	if x>=8 or x<=3 or y>=8 or y<=3:
		print("Uh oh! Watch out for the wall!")

		if x>=8:
		    desired_theta = np.pi

		elif x<=3:
		    desired_theta = 0

		elif y>=8:
		    desired_theta = 3*np.pi/2

		elif y<=3:
		    desired_theta = np.pi/2

		angle_difference = desired_theta-theta
		i   = np.sign(angle_difference)
		if abs(angle_difference) > np.pi:
			i = -i
	return i  # i will be used to nudge the turtle's angle away from the wall

def wander(turtle_name):

	global theta

	rospy.Subscriber(turtle_name+'/pose',Pose,pose_callback)
	rospy.Subscriber('/hlpr_speech_commands',String,command_callback)

	pub = rospy.Publisher(turtle_name+'/cmd_vel', Twist, queue_size=10)
	rospy.init_node('my_driver', anonymous=True)
	rate = rospy.Rate(1) # hz

	raw_input("Press enter to begin the strolling.")
	while not rospy.is_shutdown():

		# stop strolling if you've summoned a patronus. They are only so good at locating you...
		if command.find("COME HERE") >= 0:
			lin = Vector3(0,0,0)
			ang = Vector3(0,0,0)
		else:
			i = check_for_walls(turtle_name)
			lin = Vector3(1,0,0)
			ang = Vector3(0,0,(np.random.choice([-1,1],1)[0])*np.random.rand(1,1)[0][0] + i)

		stumble = Twist(lin,ang)
		pub.publish(stumble)
		rate.sleep()

if __name__ == '__main__':

	rospy.wait_for_service('turtle1/set_pen')
	try:
		changepen = rospy.ServiceProxy('turtle1/set_pen',SetPen)
		changepen(255,255,255,1,255)
	except rospy.ServiceException, e:
		print "%s"%e

	# Spawn friend turtle, Jon
	rospy.wait_for_service('spawn')
	new_turtle = rospy.ServiceProxy('spawn',Spawn)
	try:
		jon = rospy.ServiceProxy('spawn',Spawn)
		resp = new_turtle(10.5,10.5,np.pi,'jon')
	except rospy.ServiceException, e:
		print "%s"%e

	wander('turtle1')

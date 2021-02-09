#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Point
import time
from readchar import readkey, key


class MoveDrone:

	def __init__(self):
		self.tracking = False
		self.coord_sub = rospy.Subscriber("deltas",Point,self.callback)
		self.takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=100) # TODO put the takeoff topic name here
		self.landing_pub = rospy.Publisher("/ardrone/land", Empty, queue_size=100) # TODO put the landing topic name here
		self.reset_pub = rospy.Publisher("/ardrone/reset", Empty, queue_size=100) # TODO put the landing topic name here
		self.move_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100) # Publish commands to drone 


	def move_drone(self, speed=[0.0, 0.0, 0.0], orient=[0.0, 0.0, 0.0]):

		vel_msg = Twist()

		# TODO: fill the velocity fields here with the desired values
		vel_msg.linear.x = speed[0]
		vel_msg.linear.y = speed[1]
		vel_msg.linear.z = speed[2]

		#TODO: fill the angulare velocities here with the desired values

		vel_msg.angular.x = orient[0]
		vel_msg.angular.y = orient[1]
		vel_msg.angular.z = orient[2]


		self.move_pub.publish(vel_msg)

		return 0



	def takeoff_drone(self):

		empty_msg = Empty()
		self.takeoff_pub.publish(empty_msg)


	def land_drone(self):

		empty_msg = Empty()
		self.landing_pub.publish(empty_msg)
	
	def reset_drone(self):
		
		empty_msg = Empty()
		self.reset_pub.publish(empty_msg)

	def callback(self,data):
		if self.tracking:
			delta_x = data.x
			delta_y = data.y
			self.pid(delta_x, delta_y)

	def pid(self, delta_x, delta_y):
		vel_y = delta_x / 1000
		vel_z = (delta_y-200) / 1000
		print(vel_y,vel_z)
		self.move_drone([0,vel_y,vel_z],[0,0,0])


if __name__ == '__main__':

	rospy.init_node('drone_controller', anonymous=True)

	move = MoveDrone()


	# TODO define your time counter here !

	move.reset_drone()
	rospy.sleep(2)
	print('test')
	while(True):
		keyboard_input = readkey()
		print(keyboard_input)
		if keyboard_input == 't':
			move.takeoff_drone()
		elif keyboard_input == 'l':
			move.land_drone()
		elif keyboard_input == 'z':
			move.move_drone(speed=[0.15, 0.0, 0.0], orient=[0.0, 0.0, 0.0])
		elif keyboard_input == 's':
			move.move_drone(speed=[-0.15, 0.0, 0.0], orient=[0.0, 0.0, 0.0])
		elif keyboard_input == 'q':
			move.move_drone(speed=[0, 0.15, 0.0], orient=[0.0, 0.0, 0.0])
		elif keyboard_input == 'd':
			move.move_drone(speed=[0, -0.15, 0.0], orient=[0.0, 0.0, 0.0])
		elif keyboard_input == 'k':
			move.tracking = not move.tracking
			print(move.tracking)
		elif keyboard_input == 'u':
			move.move_drone(speed=[0, 0, 0.15], orient=[0.0, 0.0, 0.0])
		elif keyboard_input == 'j':
			move.move_drone(speed=[0, 0, -0.15], orient=[0.0, 0.0, 0.0])
		elif keyboard_input == 'a':
			move.move_drone(speed=[0, 0, 0], orient=[0.0, 0.0, 0.2])
		elif keyboard_input == 'e':
			move.move_drone(speed=[0, 0, -0.15], orient=[0.0, 0.0, -0.2])

		keyboard_input = 'n'
		


	








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

class Tracking:

	def __init__(self):
		print('test')
	    	self.coord_sub = rospy.Subscriber("deltas",Point,self.callback)
		# Movement pub
		self.takeoff_pub = rospy.Publisher("/ardrone/takeoff", Empty, queue_size=100) # TODO put the takeoff topic name here
		self.landing_pub = rospy.Publisher("/ardrone/land", Empty, queue_size=100) # TODO put the landing topic name here
		self.reset_pub = rospy.Publisher("/ardrone/reset", Empty, queue_size=100) # TODO put the landing topic name here
		self.move_pub = rospy.Publisher("/cmd_vel", Twist, Empty, queue_size=100)

	def callback(self,data):
		try:
			delta_x = data.x
			delta_y = data.y
			self.pid(delta_x, delta_y)
		except KeyError:
			pass


	def pid2(self, delta_x, delta_y):
		if delta_x > 10:
			self.move_drone([0,0.05,0],[0,0,0])
			#print("gauche")
		if delta_x < -10:
			self.move_drone([0,-0.05,0],[0,0,0])
			#print("droite")
		if delta_y > 210:
			self.move_drone([0,0,0.05],[0,0,0])
			#print("haut")
		if delta_y < 190:
			self.move_drone([0,0,-0.05],[0,0,0])
			#print("bas")
		else:
			self.move_drone([0,0,0],[0,0,0])

	def pid(self, delta_x, delta_y):
		vel_y = delta_x / 1000
		vel_z = (delta_y-200) / 1000
		
		self.move_drone([0,vel_y,vel_z],[0,0,0])
##############################################################################
##############################################################################
##############################################################################


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
		print("takeoff")
		empty_msg = Empty()
		self.takeoff_pub.publish(empty_msg)


	def land_drone(self):

		empty_msg = Empty()
		self.landing_pub.publish(empty_msg)

	def reset_drone(self):
		print("reset")
		empty_msg = Empty()
		self.reset_pub.publish(empty_msg)

##############################################################################
##############################################################################
##############################################################################

def main(args):
	rospy.init_node('tracking', anonymous=True)
	tr = Tracking()
	tr.reset_drone()
	tr.takeoff_drone()
	print('node traking initiated')
	try:
		rospy.spin()
	except KeyboardInterrupt:
		tr.land_drone()
		tr.reset_drone()
    		print("Shutting down")



if __name__ == '__main__':
	main(sys.argv)

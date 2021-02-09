#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String, Empty
#from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Twist
#from geometry_msgs.msg import Twist

class image_converter:

	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.callback)
		self.coord_pub = rospy.Publisher("deltas", Point, queue_size=10)
	
	def face_detection(self, image):
		face_cascade = cv2.CascadeClassifier('/root/catkin_ws/src/ardrone-facetracker/scripts/haarcascade_frontalface_default.xml')
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray, 1.3, 5)

		if len(faces) > 0:
			(x, y, w, h) = self.front_face(faces)
			cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
			#print('center coordinates : {} {} ; width : {} ; height : {}'.format(x, y, w, h))
			cv2.rectangle(image, (image.shape[1]/2 -h/2, image.shape[0]/2 - w/2), (image.shape[1]/2 + h/2, image.shape[0]/2 + w/2), (0, 255, 0), 2)
			#print(image.shape[0]/2,image.shape[1]/2,x,y)
			self.compute_deltas(image.shape[0]/2,image.shape[1]/2,x,y)
      
		cv2.imshow('img', image)
		cv2.waitKey(3)


	def front_face(self, faces):
		def area(box):
			(x, y, w, h) = box
			return w * h

		return faces[list(map(area, faces)).index(max(map(area, faces)))]

	def compute_deltas(self, x_image_center, y_image_center, x_box_center, y_box_center):
		delta_x = int(x_image_center - x_box_center)
		delta_y = int(y_image_center - y_box_center)	
		self.send_deltas(delta_x, delta_y)

	def callback(self,data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			self.face_detection(image)
		except CvBridgeError as e:
      			print(e)

	def send_deltas(self, delta_x, delta_y):
		coord_msg = Point(delta_x, delta_y,0)
		self.coord_pub.publish(coord_msg)

#    try:
#      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#    except CvBridgeError as e:
#      print(e)






def main(args):
	rospy.init_node('image_reader', anonymous=True)
	ic = image_converter()
	print('node image_reader initiated')
	try:
		rospy.spin()
	except KeyboardInterrupt:
    		print("Shutting down")
	cv2.destroyAllWindows()



if __name__ == '__main__':
	main(sys.argv)

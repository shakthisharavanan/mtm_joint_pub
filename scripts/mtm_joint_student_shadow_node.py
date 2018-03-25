#!/usr/bin/env python

import rospy
import cv2
import numpy as np

import std_msgs.msg
import sensor_msgs.msg
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from mtm_joint_pub.msg import array

class mtm_student(object):

	def __init__(self):
		rospy.init_node('mtm_student', anonymous=True)

		self.switch_value = 0

		self.outer_yaw = Float64()
		self.shoulder_pitch = Float64()
		self.elbow_pitch = Float64()
		self.wrist_platform = Float64()
		self.wrist_pitch = Float64()
		self.wrist_yaw = Float64()
		self.wrist_roll = Float64()

		self.out_yaw_pub = rospy.Publisher('/mtm_student/right_outer_yaw_joint/SetPositionTarget', Float64, queue_size=10)
		self.shoulder_pub = rospy.Publisher('/mtm_student/right_shoulder_pitch_joint/SetPositionTarget', Float64, queue_size=10)
		self.elbow_pub = rospy.Publisher('/mtm_student/right_elbow_pitch_joint/SetPositionTarget', Float64, queue_size=10)
		self.platform_pub = rospy.Publisher('/mtm_student/right_wrist_platform_joint/SetPositionTarget', Float64, queue_size=10)
		self.pitch_pub = rospy.Publisher('/mtm_student/right_wrist_pitch_joint/SetPositionTarget', Float64, queue_size=10)
		self.yaw_pub = rospy.Publisher('/mtm_student/right_wrist_yaw_joint/SetPositionTarget', Float64, queue_size=10)
		self.roll_pub = rospy.Publisher('/mtm_student/right_wrist_roll_joint/SetPositionTarget', Float64, queue_size=10)
		
		rospy.Subscriber('/mtm_teacher/joint/states', JointState, self.master_states_callback, queue_size=10)
		rospy.Subscriber('/mtm_teacher_switch', Int32, self.switch_callback, queue_size=10)

		self.rate = rospy.Rate(1000)
	
	def nothing(self, x):
		pass

	def deg2rad(self, deg):
		rad = 3.14159*deg/180
		return rad

	def rad2deg(self, rad):
		deg = 180*rad/3.14
		return int(deg)

	def master_states_callback(self, states):
		self.outer_yaw.data = states.position[0] # outer_yaw
		self.shoulder_pitch.data = states.position[1] # shoulder_pitch
		self.elbow_pitch.data = states.position[2] # elbow_pitch
		self.wrist_platform.data = states.position[3] # wrist_platform
		self.wrist_pitch.data = states.position[4] # wrist_pitch
		self.wrist_yaw.data = states.position[5] # wrist_yaw
		self.wrist_roll.data = states.position[6] # wrist_roll

	def switch_callback(self, data):
		self.switch_value = data.data

	def spin(self):

		img = np.zeros((300,512,3), np.uint8)
		cv2.namedWindow('Joint Angle Values')

		cv2.createTrackbar('Outer Yaw (75)',      'Joint Angle Values', 0, 45+75, self.nothing) #75
		cv2.createTrackbar('Shoulder Pitch (45)', 'Joint Angle Values', 0, 45+45, self.nothing) #45
		cv2.createTrackbar('Elbow Pitch (45)',    'Joint Angle Values', 0, 45+45, self.nothing) #45
		cv2.createTrackbar('Wrist Platform (60)', 'Joint Angle Values', 0, 244+60, self.nothing) #60
		cv2.createTrackbar('Wrist Pitch (90)',    'Joint Angle Values', 0, 180+90, self.nothing) #90
		cv2.createTrackbar('Wrist Yaw (45)',      'Joint Angle Values', 0, 45+45, self.nothing) #45
		cv2.createTrackbar('Wrist Roll (270)',    'Joint Angle Values', 0, 270+270, self.nothing) #270
			
		cv2.setTrackbarPos('Outer Yaw (75)', 'Joint Angle Values', 75)
		cv2.setTrackbarPos('Shoulder Pitch (45)', 'Joint Angle Values', 45)
		cv2.setTrackbarPos('Elbow Pitch (45)', 'Joint Angle Values', 45)
		cv2.setTrackbarPos('Wrist Platform (60)', 'Joint Angle Values', 60)
		cv2.setTrackbarPos('Wrist Pitch (90)', 'Joint Angle Values', 90)
		cv2.setTrackbarPos('Wrist Yaw (45)', 'Joint Angle Values', 45)
		cv2.setTrackbarPos('Wrist Roll (270)', 'Joint Angle Values', 270)

		while not rospy.is_shutdown():
			if self.switch_value == 0:
				img[:] = [0, 255, 0]
				cv2.imshow('Joint Angle Values',img)
				k = cv2.waitKey(1) & 0xFF
				if k==27:
					break

				self.outer_yaw.data = cv2.getTrackbarPos('Outer Yaw (75)', 'Joint Angle Values')
				self.shoulder_pitch.data = cv2.getTrackbarPos('Shoulder Pitch (45)', 'Joint Angle Values')
				self.elbow_pitch.data = cv2.getTrackbarPos('Elbow Pitch (45)', 'Joint Angle Values')
				self.wrist_platform.data = cv2.getTrackbarPos('Wrist Platform (60)', 'Joint Angle Values')
				self.wrist_pitch.data = cv2.getTrackbarPos('Wrist Pitch (90)', 'Joint Angle Values')
				self.wrist_yaw.data = cv2.getTrackbarPos('Wrist Yaw (45)', 'Joint Angle Values')
				self.wrist_roll.data = cv2.getTrackbarPos('Wrist Roll (270)', 'Joint Angle Values')
			   
				self.outer_yaw.data = self.deg2rad(self.outer_yaw.data-75)             
				self.shoulder_pitch.data = self.deg2rad(self.shoulder_pitch.data-45)  
				self.elbow_pitch.data = self.deg2rad(self.elbow_pitch.data-45) 
				self.wrist_platform.data = self.deg2rad(self.wrist_platform.data-60)              
				self.wrist_pitch.data = self.deg2rad(self.wrist_pitch.data-90)          
				self.wrist_yaw.data = self.deg2rad(self.wrist_yaw.data-45)             
				self.wrist_roll.data = self.deg2rad(self.wrist_roll.data-270)            
				
				self.out_yaw_pub.publish(self.outer_yaw)
				self.shoulder_pub.publish(self.shoulder_pitch)
				self.elbow_pub.publish(self.elbow_pitch)
				self.platform_pub.publish(self.wrist_platform)
				self.pitch_pub.publish(self.wrist_pitch)
				self.yaw_pub.publish(self.wrist_yaw)
				self.roll_pub.publish(self.wrist_roll)
			   
			else:
				img[:] = [0, 0, 255]
				cv2.imshow('Joint Angle Values',img)
				k = cv2.waitKey(1) & 0xFF
				if k==27:
					break

				cv2.setTrackbarPos('Outer Yaw (75)', 'Joint Angle Values', self.rad2deg(self.outer_yaw.data) + 75)
				cv2.setTrackbarPos('Shoulder Pitch (45)', 'Joint Angle Values', self.rad2deg(self.shoulder_pitch.data) + 45)
				cv2.setTrackbarPos('Elbow Pitch (45)', 'Joint Angle Values', self.rad2deg(self.elbow_pitch.data) + 45)
				cv2.setTrackbarPos('Wrist Platform (60)', 'Joint Angle Values', self.rad2deg(self.wrist_platform.data) + 60)
				cv2.setTrackbarPos('Wrist Pitch (90)', 'Joint Angle Values', self.rad2deg(self.wrist_pitch.data) + 90)
				cv2.setTrackbarPos('Wrist Yaw (45)', 'Joint Angle Values', self.rad2deg(self.wrist_yaw.data) + 45)
				cv2.setTrackbarPos('Wrist Roll (270)', 'Joint Angle Values', self.rad2deg(self.wrist_roll.data) + 270)
			   
				self.out_yaw_pub.publish(self.outer_yaw)
				self.shoulder_pub.publish(self.shoulder_pitch)
				self.elbow_pub.publish(self.elbow_pitch)
				self.platform_pub.publish(self.wrist_platform)
				self.pitch_pub.publish(self.wrist_pitch)
				self.yaw_pub.publish(self.wrist_yaw)
				self.roll_pub.publish(self.wrist_roll)
			   
			self.rate.sleep()

if __name__ == '__main__':
	mtm_student_obj = mtm_student()
	try:
		mtm_student_obj.spin()
	except rospy.ROSInterruptException:
		print 'Shutting Down'

cv2.destroyAllWindows()



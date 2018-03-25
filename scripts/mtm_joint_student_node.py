#!/usr/bin/env python

import rospy
import cv2
import numpy as np

import std_msgs.msg
from std_msgs.msg import Float64
from mtm_joint_pub.msg import array

def nothing(x):
    pass

def deg2rad(deg):
    rad = 3.14*deg/180
    return rad

if __name__ == '__main__':
    print "main"
    try:
        rospy.init_node('mtm_joint_student', anonymous=True)
        
        out_yaw_pub = rospy.Publisher('/mtm_student/right_outer_yaw_joint/SetPosition', Float64, queue_size=10)
        shoulder_pub = rospy.Publisher('/mtm_student/right_shoulder_pitch_joint/SetPosition', Float64, queue_size=10)
        elbow_pub = rospy.Publisher('/mtm_student/right_elbow_pitch_joint/SetPosition', Float64, queue_size=10)
        platform_pub = rospy.Publisher('/mtm_student/right_wrist_platform_joint/SetPosition', Float64, queue_size=10)
        pitch_pub = rospy.Publisher('/mtm_student/right_wrist_pitch_joint/SetPosition', Float64, queue_size=10)
        yaw_pub = rospy.Publisher('/mtm_student/right_wrist_yaw_joint/SetPosition', Float64, queue_size=10)
        roll_pub = rospy.Publisher('/mtm_student/right_wrist_roll_joint/SetPosition', Float64, queue_size=10)
        
        rate = rospy.Rate(100)
        img = np.zeros((300,512,3), np.uint8)
        cv2.namedWindow('Joint Angle Values')

        cv2.createTrackbar('Outer Yaw (75)',      'Joint Angle Values', 0, 45+75, nothing) #75
        cv2.createTrackbar('Shoulder Pitch (45)', 'Joint Angle Values', 0, 45+45, nothing) #45
        cv2.createTrackbar('Elbow Pitch (45)',    'Joint Angle Values', 0, 45+45, nothing) #45
        cv2.createTrackbar('Wrist Platform (60)', 'Joint Angle Values', 0, 244+60, nothing) #60
        cv2.createTrackbar('Wrist Pitch (90)',    'Joint Angle Values', 0, 180+90, nothing) #90
        cv2.createTrackbar('Wrist Yaw (45)',      'Joint Angle Values', 0, 45+45, nothing) #45
        cv2.createTrackbar('Wrist Roll (270)',    'Joint Angle Values', 0, 270+270, nothing) #270
    
        cv2.setTrackbarPos('Outer Yaw (75)', 'Joint Angle Values', 75)
        cv2.setTrackbarPos('Shoulder Pitch (45)', 'Joint Angle Values', 45)
        cv2.setTrackbarPos('Elbow Pitch (45)', 'Joint Angle Values', 45)
        cv2.setTrackbarPos('Wrist Platform (60)', 'Joint Angle Values', 60)
        cv2.setTrackbarPos('Wrist Pitch (90)', 'Joint Angle Values', 90)
        cv2.setTrackbarPos('Wrist Yaw (45)', 'Joint Angle Values', 45)
        cv2.setTrackbarPos('Wrist Roll (270)', 'Joint Angle Values', 270)
        
        outer_yaw = Float64()
        shoulder_pitch = Float64()
        elbow_pitch = Float64()
        wrist_platform = Float64()
        wrist_pitch = Float64()
        wrist_yaw = Float64()
        wrist_roll = Float64()
       
        print ('track bar create ...')
        while not rospy.is_shutdown():
            img[:] = [0,0,255]
            cv2.imshow('Joint Angle Values',img)
            k = cv2.waitKey(1) & 0xFF
            if k==27:
                break
            
            outer_yaw.data = cv2.getTrackbarPos('Outer Yaw (75)', 'Joint Angle Values')
            shoulder_pitch.data = cv2.getTrackbarPos('Shoulder Pitch (45)', 'Joint Angle Values')
            elbow_pitch.data = cv2.getTrackbarPos('Elbow Pitch (45)', 'Joint Angle Values')
            wrist_platform.data = cv2.getTrackbarPos('Wrist Platform (60)', 'Joint Angle Values')
            wrist_pitch.data = cv2.getTrackbarPos('Wrist Pitch (90)', 'Joint Angle Values')
            wrist_yaw.data = cv2.getTrackbarPos('Wrist Yaw (45)', 'Joint Angle Values')
            wrist_roll.data = cv2.getTrackbarPos('Wrist Roll (270)', 'Joint Angle Values')
           
            outer_yaw.data = deg2rad(outer_yaw.data-75)             
            shoulder_pitch.data = deg2rad(shoulder_pitch.data-45)  
            elbow_pitch.data = deg2rad(elbow_pitch.data-45) 
            wrist_platform.data = deg2rad(wrist_platform.data-60)              
            wrist_pitch.data = deg2rad(wrist_pitch.data-90)          
            wrist_yaw.data = deg2rad(wrist_yaw.data-45)             
            wrist_roll.data = deg2rad(wrist_roll.data-270)            
            
            out_yaw_pub.publish(outer_yaw)
            shoulder_pub.publish(shoulder_pitch)
            elbow_pub.publish(elbow_pitch)
            platform_pub.publish(wrist_platform)
            pitch_pub.publish(wrist_pitch)
            yaw_pub.publish(wrist_yaw)
            roll_pub.publish(wrist_roll)
           
            rate.sleep()

    except KeyboardInterrupt:
        print ("Shutting Down")

cv2.destroyAllWindows()



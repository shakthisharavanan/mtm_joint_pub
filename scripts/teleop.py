#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from mtm_joint_pub.msg import array

# def talker():
#     pub = rospy.Publisher('right_elbow_pitch', Float64, queue_size=10)
    
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

def listener():
    rospy.init_node('mtm', anonymous=True)
    rospy.Subscriber('mtm_joint_val', array, listen_cb, queue_size = 10)
    rospy.spin()

def listen_cb(data):
    rate = rospy.Rate(10) # 10hz
    print "recieved"
    pub1 = rospy.Publisher('/mtm/right_elbow_pitch_joint/SetPosition', Float64, queue_size=10)
    pub2 = rospy.Publisher('/mtm/right_outer_yaw_joint/SetPosition', Float64, queue_size=10)
    pub3 = rospy.Publisher('/mtm/right_shoulder_pitch_joint/SetPosition', Float64, queue_size=10)
    pub4 = rospy.Publisher('/mtm/right_wrist_pitch_joint/SetPosition', Float64, queue_size=10)
    pub5 = rospy.Publisher('/mtm/right_wrist_platform_joint/SetPosition', Float64, queue_size=10)
    pub6 = rospy.Publisher('/mtm/right_wrist_roll_joint/SetPosition', Float64, queue_size=10)
    pub7 = rospy.Publisher('/mtm/right_wrist_yaw_joint/SetPosition', Float64, queue_size=10)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub1.publish(data.data[0])
    pub2.publish(data.data[1])
    pub3.publish(data.data[2])
    pub4.publish(data.data[3])
    pub5.publish(data.data[4])
    pub6.publish(data.data[5])
    pub7.publish(data.data[6])
    rate.sleep()



if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass


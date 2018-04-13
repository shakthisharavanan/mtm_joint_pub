#!/usr/bin/env python
# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import tf

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""
x = 0.0
y = 0.0
z = 0.0
r = 0.0
p = 0.0
y = 0.0

moveBindings = {
		'w':(0.1,0,0,0,0,0),
		't':(0,0.1,0,0,0,0),
		'i':(0,0,0.1,0,0,0),
		'd':(0,0,0,0.1,0,0),
		'h':(0,0,0,0,0.1,0),
		'l':(0,0,0,0,0,0.1),
		's':(-0.1,0,0,0,0,0),
		'a':(0,0,0,-0.1,0,0),
		'g':(0,-0.1,0,0,0,0),
		'f':(0,0,0,0,-0.1,0),
		'k':(0,0,-0.1,0,0,0),
		'j':(0,0,0,0,0,-0.1)}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def pose_cb(values):
	global x, y, z, r, p, y
	x = values.position.x
	y = values.position.y
	z = values.position.z
	q0 = values.orientation.x
	q1 = values.orientation.y
	q2 = values.orientation.z
	q3 = values.orientation.w
	(r, p, y) = tf.transformations.euler_from_quaternion([q0, q1, q2, q3])


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('mtm_student_cartesian_pose_command', Pose, queue_size = 10)
	rospy.init_node('teleop_dvrk_keyboard')
	rospy.Subscriber('mtm_student_cartesian_pose_current', Pose, pose_cb, queue_size = 10)

	# speed = rospy.get_param("~speed", 0.5)
	# turn = rospy.get_param("~turn", 1.0)
	# x = 0
	# y = 0
	# z = 0
	# th = 0
	# status = 0

	try:
		# print msg
		# print vels(speed,turn)
		while(1):
			key = getKey()
			pose_cmd = Pose()
			# print key
			if key in moveBindings.keys():
				x_ = x + moveBindings[key][0]
				y_ = y + moveBindings[key][1]
				z_ = z + moveBindings[key][2]
				r_ = r + moveBindings[key][3]
				p_ = p + moveBindings[key][4]
				y_ = y + moveBindings[key][5]

				pose_cmd.position.x = x_
				pose_cmd.position.y = y_
				pose_cmd.position.z = z_
				quaternion = tf.transformations.quaternion_from_euler(r_, p_, y_)
				pose_cmd.orientation.x = quaternion[0]
				pose_cmd.orientation.y = quaternion[1]
				pose_cmd.orientation.z = quaternion[2]
				pose_cmd.orientation.w = quaternion[3]


			# 	print vels(speed,turn)
			# 	if (status == 14):
			# 		print msg
			# 	status = (status + 1) % 15
			else:
				pose_cmd.position.x = x
				pose_cmd.position.y = y
				pose_cmd.position.z = z
				quaternion = tf.transformations.quaternion_from_euler(r, p, y)
				pose_cmd.orientation.x = quaternion[0]
				pose_cmd.orientation.y = quaternion[1]
				pose_cmd.orientation.z = quaternion[2]
				pose_cmd.orientation.w = quaternion[3]
			# 	x = 0
			# 	y = 0
			# 	z = 0
			# 	th = 0
			if (key == '\x03'):
				break

			# twist = Twist()
			




			# twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
			# twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
			pub.publish(pose_cmd)

	except:
		# print e
		pass

	finally:
		# twist = Twist()
		# twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		# twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		# pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

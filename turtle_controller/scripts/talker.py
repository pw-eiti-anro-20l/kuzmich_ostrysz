#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSANOW, termios.tcgetattr(sys.stdin.fileno()))
    return key

def talker():
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) #10Hz
    twist = Twist()
    while not rospy.is_shutdown():
        if getKey() == rospy.get_param("/forward"):
            twist.linear.x = 1
	    twist.angular.z = 0
        elif getKey() == rospy.get_param("/backward"):
            twist.linear.x = -1
	    twist.angular.z = 0
        elif getKey() == rospy.get_param("/left"):
            twist.angular.z = 1
            twist.linear.x = 0
        elif getKey() == rospy.get_param("/right"):
            twist.angular.z = -1
            twist.linear.x = 0
        elif getKey() == "\x03":
            rospy.signal_shutdown("Pressed CTRL-c. Killing node.")
	else:
            twist.linear.x = 0
            twist.angular.z = 0
	pub.publish(twist)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

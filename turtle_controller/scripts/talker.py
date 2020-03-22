#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys, select, termios, tty

def getKey():
    fd = sys.stdin.fileno()
    newattr = termios.tcgetattr(fd)
    newattr[3] &= ~termios.ICANON
    newattr[3] &= ~termios.ECHO
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSANOW, newattr)
    reset()
    return key
 
def reset():
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
	termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, termios.tcgetattr(sys.stdin.fileno()))
	
def talker():
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) #10Hz
    twist = Twist()
    while not rospy.is_shutdown():
        if getKey() == rospy.get_param("/forward"):
            twist.linear.x = 2
	    twist.angular.z = 0
        elif getKey() == rospy.get_param("/backward"):
            twist.linear.x = -2
	    twist.angular.z = 0
        elif getKey() == rospy.get_param("/left"):
            twist.angular.z = 2
            twist.linear.x = 0
        elif getKey() == rospy.get_param("/right"):
            twist.angular.z = -2
            twist.linear.x = 0
        elif getKey() == "\x03":
            raise KeyboardInterrupt
	else:
            twist.linear.x = 0
            twist.angular.z = 0
	rospy.loginfo(twist)
	pub.publish(twist)
	rate.sleep()	

if __name__ == '__main__':
    try:
	newattr = termios.tcgetattr(sys.stdin)
        talker()
    except rospy.ROSInterruptException:
        pass
    finally:
	reset()
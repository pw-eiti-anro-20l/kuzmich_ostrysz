#!/usr/bin/env python

import rospy
import json
import os
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import *
from tf.transformations import *

params = {}
with open(os.path.dirname(os.path.realpath(__file__)) + '/../mdh_value.json', 'r') as file:
    params = json.loads(file.read())


def inverted_kinematics(data):
    global alpha1
    global alpha2 

    x=data.pose.position.x
    y=data.pose.position.y
    z=data.pose.position.z
    if x == 0 and y == 0:
        x=1
        y=1
	z=sqrt((x^2)+(y^2))
    a1,d2,alpha1,th2=params["i2"]
    a2,d3,alpha2,th3=params["i3"]
    z3 = d3-z

    if x**2+y**2 > a1**2+a2**2:
        rospy.logerr("Error. Incorrect position")
        return {"status": False, "message": "Incorrect position"}

    alpha22=-acos((x**2+y**2-a1**2-a2**2)/(2*a1*a2))
    alpha12=asin((a2*sin(alpha22))/(sqrt(x**2+y**2)))+atan2(y,x)
    
    alpha21=acos((x**2+y**2-a1**2-a2**2)/(2*a1*a2))
    alpha11=-asin((a2*sin(alpha21))/(sqrt(x**2+y**2)))+atan2(y,x)
    if fabs(alpha1-alpha11)>fabs(alpha1-alpha12) :
        alpha1=alpha12
        alpha2=alpha22
    else :
        alpha1=alpha11
        alpha2=alpha21  

    computed_joint_state = JointState()
    computed_joint_state.header = Header()
    computed_joint_state.header.stamp = rospy.Time.now()
    computed_joint_state.name = ['base_link_to_link1', 'link1_to_link2', 'link2_to_link3']
    computed_joint_state.position = [alpha1, alpha2, z3]
    computed_joint_state.velocity = []
    computed_joint_state.effort = []
    jstate_pub.publish(computed_joint_state)

if __name__ == "__main__":
    rospy.init_node('ikin_node')
    sub = rospy.Subscriber('interpolation', PoseStamped, inverted_kinematics)
    jstate_pub = rospy.Publisher('jstate', JointState, queue_size=10)
    rospy.spin()

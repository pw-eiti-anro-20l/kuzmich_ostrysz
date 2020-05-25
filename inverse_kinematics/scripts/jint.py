#!/usr/bin/env python

import rospy
import time
from Interpolation.srv import Interpolation
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

current_pos = [0,0,0]
freq = 50


def handle_interpolation(data):
    if data.t <= 0 or not -1.57075 <= data.j1 <= 1.57075 or not -1.57075 <= data.j2 <= 1.57075 or not 0 <= data.j3 <= 0.2:
        return False

    new_pos = [data.j1, data.j2, data.j3]
    rate = rospy.Rate(freq)
    j1, j2, j3 = current_pos[0], current_pos[1], current_pos[2]
    frames_number = int(math.ceil(data.t * freq))
    current_time = 0.

    for k in range(0, frames_number + 1):

        current_pos[0] = compute_int(j1, new_pos[0], data.t, current_time, data.i)
        current_pos[1] = compute_int(j2, new_pos[1], data.t, current_time, data.i)
        current_pos[2] = compute_int(j3, new_pos[2], data.t, current_time, data.i)
        current_time = current_time + 1. / freq
        rate.sleep()

    return True


def compute_int(start_j, last_j, time, current_time, i):
    if i == 'complex':
        return compute_complex(start_j, last_j, time, current_time)
    else:
        return compute_const(start_j, last_j, time, current_time)


def compute_const(start_j, last_j, time, current_time):
    return start_j + (float(last_j - start_j) / time) * current_time


def compute_complex(start_j, last_j, time, current_time):
    h = 2. * float(last_j - start_j) / time
    ratio = h / (time / 2.)
    if current_time < time / 2.:
        return start_j + current_time**2 * ratio / 2.
    else:
        return last_j - (time-current_time)**2 * ratio / 2.

def jointPublisher():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    srv = rospy.Service('int', Interpolation, handle_interpolation)
    rospy.init_node('int_srv')
    rate = rospy.Rate(100)
    joint_state = JointState()
    while not rospy.is_shutdown():
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['base_link_to_link1', 'link1_to_link2', 'link2_to_link3']
        joint_state.position = current_pos
        joint_state.velocity = [0,0,0]
        joint_state.effort = [0,0,0]
        pub.publish(joint_state)

if __name__ == "__main__":
    try:
        jointPublisher()
    except rospy.ROSInterruptException:
        pass

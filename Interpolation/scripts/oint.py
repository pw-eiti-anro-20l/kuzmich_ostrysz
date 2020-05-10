#!/usr/bin/env python

import rospy
from Interpolation.srv import Oint
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math

#wartosci poczatkowe
frequency = 50
current_trans = [0, 0, 0]
current_rot = [0, 0, 0, 1]

#funkcja interpolujaca
def handleInterpolation(data):
    #sprawdzenie danych
    if data.time <= 0.:
        return False
    #zapisanie danych o ukladach i interpolacji
    x, y, z = current_trans[0], current_trans[1], current_trans[2]
    qx, qy, qz = current_rot[0], current_rot[1], current_rot[2]
    new_trans = [data.x, data.y, data.z]
    new_rot = [data.qx, data.qy, data.qz]
    rate = rospy.Rate(frequency)
    current_time = 0.
    frames_number = int(math.ceil(data.time * frequency))
    #interpolacja
    for i in range(frames_number+1):
        current_trans[0] = interpolation(x, new_trans[0], data.time, current_time, data.style)
        current_trans[1] = interpolation(y, new_trans[1], data.time, current_time, data.style)
        current_trans[2] = interpolation(z, new_trans[2], data.time, current_time, data.style)
        current_rot[0] = interpolation(qx, new_rot[0], data.time, current_time, data.style)
        current_rot[1] = interpolation(qy, new_rot[1], data.time, current_time, data.style)
        current_rot[2] = interpolation(qz, new_rot[2], data.time, current_time, data.style)
        current_time = current_time + 1.0 / frequency
        rate.sleep()
    return True

#sprawdzenie rodzaju interpolacji
def interpolation(start, last, time, current_time, style):
    if style == 'complex':
        return complexInterpolation(start, last, time, current_time)
    else:
        return simpleInterpolation(start, last, time, current_time)

#interpolacja prostym sposobem
def simpleInterpolation(start, last, time, current_time):
    return start + (float(last - start) / time) * current_time

#interpolacja zlozonym sposobem
def complexInterpolation(start, last, time, current_time):
    h = 2. * float(last - start) / time
    ratio = h / (time / 2.)
    if current_time < time / 2.:
        return start + current_time**2 * ratio / 2.
    else:
        return last - (time-current_time)**2 * ratio / 2.

#funkcja publikujaca transformacje
def transformationPublisher():
    pub = rospy.Publisher('interpolation', PoseStamped, queue_size=10)
    srv = rospy.Service('int', Oint, handleInterpolation)
    rospy.init_node('int_srv')
    rate = rospy.Rate(100)
    pose = PoseStamped()
    while not rospy.is_shutdown():
        pose.header.frame_id = 'base'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = current_trans[0]
        pose.pose.position.y = current_trans[1]
        pose.pose.position.z = current_trans[2]
        pose.pose.orientation.x = current_rot[0]
        pose.pose.orientation.y = current_rot[1]
        pose.pose.orientation.z = current_rot[2]
        pose.pose.orientation.w = 1
        pub.publish(pose)

if __name__ == "__main__":
    try:
        transformationPublisher()
    except rospy.ROSInterruptException:
        pass

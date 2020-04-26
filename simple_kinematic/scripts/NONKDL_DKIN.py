#! /usr/bin/python

import rospy
import json
import os
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)


def correctPosition(data):
    if data.position[0] < borders['i1'][0] or data.position[0] > borders['i1'][1]:
        return False

    if data.position[1] < borders['i2'][0] or data.position[1] > borders['i2'][1]:
        return False

    if data.position[2] < borders['i3'][0] or data.position[2] > borders['i3'][1]:
        return False

    return True


def simple_kinematics(data):
    if not correctPosition(data):
        rospy.logerr('Incorrect position! ' + str(data))
        return

    a, d, al, th = params['i1']
    al, a, d, th = float(al), float(a), float(d), float(th)
    tz = translation_matrix((0, 0, d))
    rz = rotation_matrix(data.position[0], zaxis)
    tx = translation_matrix((a, 0, 0))
    rx = rotation_matrix(al, xaxis)
    T1 = concatenate_matrices(rx, tx, rz, tz)

    a, d, al, th = params['i2']
    al, a, d, th = float(al), float(a), float(d), float(th)
    tz = translation_matrix((0, 0, d))
    rz = rotation_matrix(data.position[1], zaxis)
    tx = translation_matrix((a, 0, 0))
    rx = rotation_matrix(al, xaxis)
    T2 = concatenate_matrices(rx, tx, rz, tz)

    a, d, al, th = params['i3']
    al, a, d, th = float(al), float(a), float(d), float(th)
    tz = translation_matrix((0, 0, data.position[2]-d))
    rz = rotation_matrix(th, zaxis)
    tx = translation_matrix((a, 0, 0))
    rx = rotation_matrix(al, xaxis)
    T3 = concatenate_matrices(rx, tx, rz, tz)

    Tk = concatenate_matrices(T1, T2, T3)
    x, y, z = translation_from_matrix(Tk)
    qx, qy, qz, qw = quaternion_from_matrix(Tk)

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('NONKDL_DKIN', anonymous=True)

    pub = rospy.Publisher('npose_stamped', PoseStamped, queue_size=10)
    rospy.Subscriber('joint_states', JointState, simple_kinematics)

    params = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../mdh_value.json', 'r') as file:
        params = json.loads(file.read())

    bounds = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../borders.json', 'r') as file:
        borders = json.loads(file.read())

    rospy.spin()

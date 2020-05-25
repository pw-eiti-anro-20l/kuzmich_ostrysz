#! /usr/bin/python

import rospy
import json
import os
import PyKDL as kdl
import math
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)


def correct_Position(data):
    if data.position[0] < borders['i1'][0] or data.position[0] > borders['i1'][1]:
        return False

    if data.position[1] < borders['i2'][0] or data.position[1] > borders['i2'][1]:
        return False

    if data.position[2] < borders['i3'][0] or data.position[2] > borders['i3'][1]:
        return False

    return True


def simple_kinematics(data):
    if not correct_Position(data):
        rospy.logerr('Incorrect position! ' + str(data))
        return

    kdlChain = kdl.Chain()
    frameFactory = kdl.Frame();

    frame0 = frameFactory.DH(0, 0, 0, 0);
    joint0 = kdl.Joint(kdl.Joint.None)
    kdlChain.addSegment(kdl.Segment(joint0, frame0))

    a, d, al, th = params['i2']
    al, a, d, th = float(al), float(a), float(d), float(th)
    frame1 = frameFactory.DH(a, al, d, th)
    joint1 = kdl.Joint(kdl.Joint.RotZ)
    kdlChain.addSegment(kdl.Segment(joint1, frame1))

    a, d, al, th = params['i1']
    al, a, d, th = float(al), float(a), float(d), float(th)
    frame2 = frameFactory.DH(a, al, d, th)
    joint2 = kdl.Joint(kdl.Joint.RotZ)
    kdlChain.addSegment(kdl.Segment(joint2, frame2))

    a, d, al, th = params['i3']
    al, a, d, th = float(al), float(a), float(d), float(th)
    frame3 = frameFactory.DH(a, al, d, th)
    joint3 = kdl.Joint(kdl.Joint.TransZ)
    kdlChain.addSegment(kdl.Segment(joint3, frame3))

    jointAngles = kdl.JntArray(kdlChain.getNrOfJoints())
    jointAngles[0] = data.position[0]
    jointAngles[1] = data.position[1]
    jointAngles[2] = -data.position[2] 

    fksolver = kdl.ChainFkSolverPos_recursive(kdlChain)
    solvedFrame = kdl.Frame()
    fksolver.JntToCart(jointAngles, solvedFrame)
    quaternion = solvedFrame.M.GetQuaternion()

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = solvedFrame.p[0]
    pose.pose.position.y = solvedFrame.p[1]
    pose.pose.position.z = solvedFrame.p[2]
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('KDL_DKIN', anonymous=True)
    pub = rospy.Publisher('pose_stamped', PoseStamped, queue_size=10)
    rospy.Subscriber('joint_states', JointState, simple_kinematics)

    params = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../mdh_value.json', 'r') as file:
        params = json.loads(file.read())

    bounds = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../borders.json', 'r') as file:
        borders = json.loads(file.read())

    rospy.spin()

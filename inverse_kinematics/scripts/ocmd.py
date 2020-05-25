#!/usr/bin/env python

import sys
import rospy
from math import *
from Interpolation.srv import Oint

#funkcja rzadajaca interpolacji ukladu
def interpolationClient(x, y, z, qx, qy, qz, time, style):
    rospy.wait_for_service('int')
    try:
        interpolation = rospy.ServiceProxy('int', Oint)
        response = interpolation(x, y, z, qx, qy, qz, time, style)
        return response.status
    except rospy.ServiceException, e:
        print "Service failed: %s"%e

def rectangle(x, y, time, style):
    rospy.wait_for_service('int')
    qx = 0
    qy = 0
    qz = 0
    z = 1
    try:
        interpolation = rospy.ServiceProxy('int', Oint)
        for _ in range (4):
            response = interpolation(x, y, z, qx, qy, qz, time, style)
            response = interpolation(-x, y, z, qx, qy, qz, time, style)
            response = interpolation(-x, -y, z, qx, qy, qz, time, style)
            response = interpolation(x, -y, z, qx, qy, qz, time, style)
        return response.status
    except rospy.ServiceException, e:
        print "Service failed: %s"%e

def elipse(x, y, style):
    rospy.wait_for_service('int')
    qx = 0
    qy = 0
    qz = 0
    z = 1
    try:
        interpolation = rospy.ServiceProxy('int', Oint)
        time = 0
        while True:
            response = interpolation(x * cos(time), y * sin(time), z, qx, qy, qz, 1.0/30, style)
            time += 0.1
        return response.status
    except rospy.ServiceException, e:
        print "Service failed: %s"%e

if __name__ == "__main__":
    #wczytanie danych interpolacji ukladu
    if len(sys.argv) == 9:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        qx = float(sys.argv[4])
        qy = float(sys.argv[5])
        qz = float(sys.argv[6])
        time = float(sys.argv[7])
        style = sys.argv[8]
        print "Requesting interpolation."
        #rozpoczecie serwisu interpolacji ukladu
        print interpolationClient(x, y, z, qx, qy, qz, time, style)
    elif len(sys.argv) == 5:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        time = float(sys.argv[3])
	style = sys.argv[4]
        rectangle(x, y, time, style)
    elif len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
	style = sys.argv[3]
        elipse(x, y, style)
    else:
        sys.exit(1)


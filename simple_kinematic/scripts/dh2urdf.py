#! /usr/bin/python

import json

from tf.transformations import *

xaxis, zaxis = (1, 0, 0), (0, 0, 1)

if __name__ == '__main__':
    params = {}
    results = ''
    with open('../mdh_value.json', 'r') as file:
        params = json.loads(file.read())

    with open('../urdf.yaml', 'w') as file:
        for key in params.keys():
            a, d, al, th = params[key]
            al, a, d, th = float(al), float(a), float(d), float(th)

            tz = translation_matrix((0, 0, d))
            rz = rotation_matrix(th, zaxis)
            tx = translation_matrix((a, 0, 0))
            rx = rotation_matrix(al, xaxis)

            matrix = concatenate_matrices(tz, rz, tx, rx)

            rpy = euler_from_matrix(matrix)
            xyz = translation_from_matrix(matrix)

            file.write(key + ":\n")
            file.write("  j_xyz: {} {} {}\n".format(*xyz))
            file.write("  j_rpy: {} {} {}\n".format(*rpy))
            file.write("  l_xyz: {} {} {}\n".format(xyz[0] / 2, xyz[1] / 2, xyz[2] /2))
	    if a != 0:
        	file.write("  l_rpy: 0 {} 0\n".format(-math.atan(d/a)))
	    else:
        	file.write("  l_rpy: 0 0 0\n")
            file.write("  l_len: {}\n".format(math.sqrt(a*a + d*d)))

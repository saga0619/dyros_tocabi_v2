#!/usr/bin/env python
from __future__ import division, print_function
import rospy
import argparse
from tocabi_msgs.msg import positionCommand

rospy.init_node('test_motion_generator')


parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--traj_time', help='trajectory time', type=float, default=5.0)

args = parser.parse_args()

traj_time = args.traj_time
wait_time = traj_time*1.05

print('traj_time',traj_time)

pub = rospy.Publisher('/tocabi/positioncommand', positionCommand, queue_size=1)
msg= positionCommand()
msg.gravity = False
msg.relative = False
msg.traj_time = traj_time

position1 = [-0.10000000149011612, 0.15000000596046448, -0.23999999463558197, 0.6000000238418579, -0.36000001430511475, -0.15000000596046448, 0.10000000149011612, -0.15000000596046448, -0.23999999463558197, 0.6000000238418579, -0.36000001430511475, 0.15000000596046448, 0.10000000149011612, 0.0, -0.1, 0.30000001192092896, 0.30000001192092896, 1.5, -1.2699999809265137, -1.0, 0.0, -1.0, 0.0, 0.0, -0.5, -0.30000001192092896, -0.30000001192092896, -1.5, 1.2699999809265137, 1.0, 0.0, 1.0, 0.0]

position2 = [0.0, 0.0, -0.9300000071525574, 1.2400000095367432, -0.5, 0.0, 0.0, 0.0, -0.9300000071525574, 1.2400000095367432, -0.5, 0.0, -0.10000000149011612, 0.4399999976158142, 0.10000000149011612, -0.38999998569488525, -1.2050000429153442, 0.5, -0.375, -1.100000023841858, 2.309999942779541, -1.1759999990463257, 0.15000000596046448, 0.0, 0.0, 0.38999998569488525, 1.2050000429153442, -0.5, 0.375, 1.100000023841858, -2.309999942779541, 1.1759999990463257, -0.15000000596046448]

r = rospy.Rate(1/wait_time)

pos_set = [position1, position2]

while rospy.is_shutdown() is False:
    for pos in pos_set:
        msg.position = pos
        pub.publish(msg)
        r.sleep() 

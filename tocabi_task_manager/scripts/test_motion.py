#!/usr/bin/env python3
from __future__ import division, print_function
import rospy
import argparse
from tocabi_msgs.msg import positionCommand

rospy.init_node('test_motion_generator')


parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('--time', help='trajectory time', type=float, default=5.0)

parser.add_argument('--mode',help='command type', type=int, default =0)

args = parser.parse_args()

traj_time = args.time

traj_mode = args.mode

wait_time = traj_time*1.05

print('traj_time',traj_time)

pub = rospy.Publisher('/tocabi/positioncommand', positionCommand, queue_size=1)
msg= positionCommand()
msg.gravity = False
msg.relative = True
msg.traj_time = traj_time

position1 = [0.0, 0.0, -0.24, 0.6, -0.36, -0.0, 0.0, 0.0, -0.44, 1.0, -0.56, 0.0, 0.0, 0.0, -0.0, 0.30, 0.30, 1.5, -1.27, -1.0, 0.0, -0.1, 0.0, 0.0, 0.0, -0.30, -0.30, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0]

position2 = [0.0, 0.0, -0.24, 0.6, -0.36, 0.0, 0.0, 0.0, -0.24, 0.6, -0.36, -0.0,  0.0, 0.0, -0.0, 0.30, 0.30, 1.5, -1.27, -1.0, 0.0, -0.1, 0.0, 0.0, 0.0, -0.30, -0.30, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0]

position3 = [0.0, 0.0, -0.44, 1.0, -0.56, 0.0, 0.0, 0.0, -0.24, 0.6, -0.36, -0.0,  0.0, 0.0, -0.0, 0.30, 0.30, 1.5, -1.27, -1.0, 0.0, -0.1, 0.0, 0.0, 0.0, -0.30, -0.30, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0]

position4 = [0.0, 0.0, -0.24, 0.6, -0.36, 0.0, 0.0, 0.0, -0.24, 0.6, -0.36, -0.0,  0.0, 0.0, -0.0, 0.30, 0.30, 1.5, -1.27, -1.0, 0.0, -0.1, 0.0, 0.0, 0.0, -0.30, -0.30, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0]

position01 = [-0.10, 0.15, -0.24, 0.60, -0.36, -0.15, 0.10, -0.15, -0.24, 0.60, -0.36, 0.15, 0.10, 0.0, -0.1, 0.30, 0.30, 1.5, -1.27, -1.0, 0.0, -1.0, 0.0, 0.0, -0.5, -0.30, -0.30, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0]

position02 = [0.0, 0.0, -0.93, 1.24, -0.5, 0.0, 0.0, 0.0, -0.93, 1.24, -0.5, 0.0, -0.10, 0.44, 0.10, -0.39, -1.205, 0.5, -0.375, -1.1, 2.31, -1.176, 0.15, 0.0, 0.0, 0.39, 1.205, -0.5, 0.375, 1.10, -2.31, 1.176, -0.15]

r = rospy.Rate(1/wait_time)
pos_set = [position01, position02]

if traj_mode == 1:
    pos_set = [position1, position2, position3, position4]

while rospy.is_shutdown() is False:
    for pos in pos_set:
        msg.position = pos
        pub.publish(msg)
        r.sleep() 

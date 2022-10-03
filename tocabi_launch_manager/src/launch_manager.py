#!/usr/bin/env python3

import rospy
import roslaunch
from std_msgs.msg import String
import subprocess

from subprocess import Popen, PIPE, TimeoutExpired
from threading import Thread
import sys
from signal import SIGINT
from datetime import datetime
import os

try:
    from queue import Queue, Empty
except ImportError:
    from Queue import Queue, Empty

ON_POSIX = 'posix' in sys.builtin_module_names



global starterSwitch
global stopperSwitch
global restarterSwitch
global roslaunch_subprocess
global res_subprocess


starterSwitch = False
stopperSwitch = False
restarterSwitch = False


def startercallback(data):
    global starterSwitch
    global restarterSwitch
    global roslaunch_subprocess
    if data.data == "start_tocabi":
        pub_syslog.publish(String('Start Signal Received'))
        print("Start Signal Rcv",datetime.now().strftime("%H:%M:%S"))
        starterSwitch = True
    if data.data == "restart_websocket":
        pub_syslog.publish(String('Restart ROSBRIDGE_WEBSOCKET'))
        print("restart websocket",datetime.now().strftime("%H:%M:%S"))
        restarterSwitch = True

        

def stoppercallback(data):
    global stopperSwitch
    if data.data == "stop_tocabi":
        pub_syslog.publish(String('Stop Signal Received'))
        print("Stop Signal Received",datetime.now().strftime("%H:%M:%S"))
        stopperSwitch = True



# launch.start()
# rospy.loginfo("started")
# rospy.sleep(3)
# launch.shutdown()
global pub_syslog


# Get stdout from subprocess and process it to queue
def enque_output(out,queue): 
    # print("start iter")
    for line in iter(out.readline, b''):
        # queue.put(line)
        pub_syslog.publish(String(str(line[:-1].decode())))
        #print(line[:-1].decode())
        # print('get line')
        print(line.decode('utf-8').rstrip())

    # print("end iter")
    out.close()

# Get stdout from subprocess and process it to queue
def enque_output_err(out,queue): 
    # print("start err iter")
    for line in iter(out.readline(), b''):
        # queue.put(line)
        pub_syslog.publish(String(str(line[:-1].decode())))
        #print(line[:-1].decode())
        # print('get err line')
        print(line.decode('utf-8').rstrip())

    print("end web iter")
    out.close()



if __name__ == '__main__':
    
    output = Popen(['uname','-r'], stdout=PIPE).communicate()[0]
    print(output.decode())

    rospy.init_node('tocabi_launchmanager')
    rospy.Subscriber("/tocabi/starter",String,startercallback)
    rospy.Subscriber("/tocabi/stopper",String,stoppercallback)

    pub_syslog = rospy.Publisher("/tocabi/syslog",String, queue_size=10)
    
    pub_launchman = rospy.Publisher("/tocabi/launchman",String, queue_size=10)

    #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #roslaunch.configure_logging(uuid)
    #cli_args = ['tocabi_controller','realrobot.launch','log:=true','hand:=true']

    rl_is_running = False
    #roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
    #roslaunch_args = cli_args[2:]
    rate = rospy.Rate(60)

    tick = 0
    q = None
    q2 = None
    t = None
    t2=None

    launch_hearbeat = 0

    while not rospy.is_shutdown():
        rate.sleep()


        launch_hearbeat = launch_hearbeat +1

        if(launch_hearbeat == 300):
            launch_hearbeat = 0
            pub_launchman.publish(String("tocabi_launchman"))
        # if(rl_is_running == True):
        #     try: line = q.get_nowait()
        #     except Empty:
        #         None
        #         #print('no input')
        #     else:
        #         pub_syslog.publish(String(str(line.decode())))
        #         print(line[:-1].decode())
        #         #print(line.decode())


        tick = tick+1


        if(starterSwitch == True):     
            
            
            rl_is_running = True
            roslaunch_subprocess = Popen(['roslaunch','tocabi_controller','realrobot.launch'],stdout=PIPE, stderr=PIPE)
            q=Queue()
            # q2=Queue()
            t= Thread(target=enque_output, args=(roslaunch_subprocess.stdout,q))

            # roslaunch_subprocess.stdout.
            # t2= Thread(target=enque_output_err, args=(roslaunch_subprocess.stderr,q))

            t.daemon = True
            # t2.daemon = True
            t.start()
            # t2.start()


            #launch = roslaunch.parent.ROSLaunchParent(uuid, (roslaunch_file, roslaunch_args))
            #launch.start()
            starterSwitch = False
        
        if(stopperSwitch == True):            
            try:
                roslaunch_subprocess.communicate(timeout=1)
            except TimeoutExpired:
                print('communicate pass')
            rl_is_running = False
            print('sending sigint',datetime.now().strftime("%H:%M:%S"))
            roslaunch_subprocess.send_signal(SIGINT)
            print('subprocess.communicate',datetime.now().strftime("%H:%M:%S"))
            roslaunch_subprocess.communicate()
            print('turned off!',datetime.now().strftime("%H:%M:%S"))
            # t.end()
            #launch.shutdown()
            stopperSwitch=False

        
        if(restarterSwitch == True):
            restarterSwitch=False
            print('restarting rosbridge_websocket')
            res_subprocess =subprocess.run(['systemctl', 'restart', 'tocabi_webserver.service'])
            
            print(res_subprocess, 'restart complete rosbridge_websocket')

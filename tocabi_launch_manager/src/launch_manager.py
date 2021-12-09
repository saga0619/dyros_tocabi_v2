#!/usr/bin/env python2.7

import rospy
import roslaunch
from std_msgs.msg import String
from subprocess import Popen, PIPE
from threading import Thread
import sys
from signal import SIGINT

try:
    from queue import Queue, Empty
except ImportError:
    from Queue import Queue, Empty

ON_POSIX = 'posix' in sys.builtin_module_names

def enque_output(out,queue):
    for line in iter(out.readline, b''):
        queue.put(line)
    out.close()


global starterSwitch
global stopperSwitch
global roslaunch_subprocess


starterSwitch = False
stopperSwitch = False


def startercallback(data):
    global starterSwitch
    global roslaunch_subprocess
    if data.data == "start_tocabi":
        print("start switch!")
        starterSwitch = True
        

def stoppercallback(data):
    global stopperSwitch
    if data.data == "stop_tocabi":
        print("stop switch!")
        stopperSwitch = True



# launch.start()

# rospy.loginfo("started")

# rospy.sleep(3)

# launch.shutdown()


if __name__ == '__main__':
    
    output = Popen(['uname','-r'], stdout=PIPE).communicate()[0]
    print(output)

    rospy.init_node('tocabi_launcher', anonymous=True)
    rospy.Subscriber("/tocabi/starter",String,startercallback)
    rospy.Subscriber("/tocabi/stopper",String,stoppercallback)
    pub_syslog = rospy.Publisher("/tocabi/syslog",String, queue_size=10)
    #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #roslaunch.configure_logging(uuid)

    #cli_args = ['tocabi_controller','realrobot.launch','log:=true','hand:=true']


    rl_is_running = False

    #roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)

    #roslaunch_args = cli_args[2:]

    rate = rospy.Rate(60)

    tick = 0
    q = None
    t = None
    while not rospy.is_shutdown():
        rate.sleep()

        if(rl_is_running == True):
            try: line = q.get_nowait()
            except Empty:
                None
                #print('no input')
            else:
                pub_syslog.publish(String(line))
                print(line[:-1])


        tick = tick+1


        if(starterSwitch == True):     
            
            
            rl_is_running = True
            roslaunch_subprocess = Popen(['roslaunch','tocabi_controller','simulation.launch'],stdout=PIPE, stderr=PIPE)
            q=Queue()
            t= Thread(target=enque_output, args=(roslaunch_subprocess.stdout,q))
            t.daemon = True
            t.start()
            #launch = roslaunch.parent.ROSLaunchParent(uuid, (roslaunch_file, roslaunch_args))
            #launch.start()
            starterSwitch = False
        
        if(stopperSwitch == True):
            rl_is_running = False
            roslaunch_subprocess.send_signal(SIGINT)
            roslaunch_subprocess.communicate()
            print('turned off!')
            #launch.shutdown()
            stopperSwitch=False

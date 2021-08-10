#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib#; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from robot_do_an.msg import Torque
from std_msgs.msg import Float64

import sys, select, termios, tty

msg= """
Reading form the keyboard and publishing to /cmd_vel_topic
-------------------------------------------------
Robot base moving:
    w
a   s   d
Robot arm manipulation:
link 1: c   v
link 2: b   n
link 3: m   ,
link 4: .   /

CTRL-C to quit
"""

moveBindings = {
    'w':(1,1,0,0,0,0),
    'a':(-1,1,0,0,0,0),
    's':(-1,-1,0,0,0,0),
    'd':(1,-1,0,0,0,0),
    'c':(0,0,1,0,0,0),
    'v':(0,0,-1,0,0,0),
    'b':(0,0,0,1,0,0),
    'n':(0,0,0,-1,0,0),
    'm':(0,0,0,0,1,0),
    ',':(0,0,0,0,-1,0),
    '.':(0,0,0,0,0,1),
    '/':(0,0,0,0,0,-1),
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        # self.publisher = rospy.Publisher('cmd_torque', Torque, queue_size=6)
        self.left_pub = rospy.Publisher('/robot_do_an/left_wheel_controller/command', Float64, queue_size=1)
        self.right_pub = rospy.Publisher('/robot_do_an/right_wheel_controller/command', Float64, queue_size=1)
        self.joint1_pub = rospy.Publisher('/robot_do_an/joint1_controller/command', Float64, queue_size=1)
        self.joint2_pub = rospy.Publisher('/robot_do_an/joint2_controller/command', Float64, queue_size=1)
        self.joint3_pub = rospy.Publisher('/robot_do_an/joint3_controller/command', Float64, queue_size=1)
        self.joint4_pub = rospy.Publisher('/robot_do_an/joint4_controller/command', Float64, queue_size=1)
        self.left_wheel = 0.0
        self.right_wheel = 0.0
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.theta3 = 0.0
        self.theta4 = 0.0
        self.speed_wheel = -100.0
        self.speed_joint = 0.05
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    # def wait_for_subscribers(self):
    #     i = 0
    #     while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
    #         if i == 4:
    #             print("Waiting for subscriber to connect to {}".format(self.publisher.name))
    #         rospy.sleep(0.5)
    #         i += 1
    #         i = i % 5
    #     if rospy.is_shutdown():
    #         raise Exception("Got shutdown request before subscribers connected")

    def update(self, left_wheel, right_wheel, theta1, theta2, theta3, theta4):
        self.condition.acquire()
        self.left_wheel = left_wheel
        self.right_wheel = right_wheel
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3
        self.theta4 = theta4
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        cmd = Torque()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            cmd.left_wheel = self.left_wheel
            cmd.right_wheel = self.right_wheel
            cmd.theta1 = self.theta1
            cmd.theta2 = self.theta2
            cmd.theta3 = self.theta3
            cmd.theta4 = self.theta4

            self.condition.release()

            # Publish.
            # self.publisher.publish(cmd)
            self.left_pub.publish(self.left_wheel*self.speed_wheel)
            self.right_pub.publish(self.right_wheel*self.speed_wheel)
            self.joint1_pub.publish(self.theta1*self.speed_joint)
            self.joint2_pub.publish(self.theta2*self.speed_joint)
            self.joint3_pub.publish(self.theta3*self.speed_joint)
            self.joint4_pub.publish(self.theta4*self.speed_joint)

        # Publish stop message when thread exits.
            cmd.left_wheel = 0
            cmd.right_wheel = 0
            cmd.theta1 = 0
            cmd.theta2 = 0
            cmd.theta3 = 0
            cmd.theta4 = 0
        # self.publisher.publish(cmd)
        self.left_pub.publish(0)
        self.right_pub.publish(0)
        self.joint1_pub.publish(0)
        self.joint2_pub.publish(0)
        self.joint3_pub.publish(0)
        self.joint4_pub.publish(0)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_control')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    left_wheel=0
    right_wheel=0
    theta1=0
    theta2=0
    theta3=0
    theta4=0
    status=0

    try:
        # pub_thread.wait_for_subscribers()
        pub_thread.update(left_wheel,right_wheel,theta1,theta2,theta3,theta4)

        print(msg)

        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                left_wheel = moveBindings[key][0]
                right_wheel = moveBindings[key][1]
                theta1 += moveBindings[key][2]
                theta2 += moveBindings[key][3]
                theta3 += moveBindings[key][4]
                theta4 += moveBindings[key][5]

                # if (status == 14):
                #     print(msg)
                # status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and left_wheel == 0 and right_wheel == 0 and theta1 == 0 and theta2 == 0 and  theta3 == 0 and theta4 == 0:
                    continue
                left_wheel=0
                right_wheel=0
                theta1=0
                theta2=0
                theta3=0
                theta4=0
                if (key == '\x03'):
                    break
            pub_thread.update(left_wheel,right_wheel,theta1,theta2,theta3,theta4)
            # pub_thread.run()

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
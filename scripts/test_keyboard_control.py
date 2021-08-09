from __future__ import print_function
from threading import Thread
import rospy
from robot_do_an.msg import Torque
from std_msgs.msg import Float64

def callback(data):
    rospy.loginfo("Messeage: %s \t %s \t %s \t %s \t %s \t %s .",data.left_wheel, data.right_wheel, data.theta1, data.theta2, data.theta3, data.theta4)
    talker(data.left_wheel*10,data.right_wheel*10)

def listener():
    rospy.init_node('test_keyboard_control', anonymous=True)
    rospy.Subscriber('cmd_torque', Torque, callback)
    rospy.spin()

def talker(data_left, data_right):
    left_pub = rospy.Publisher('/robot_do_an/left_wheel_controller/command', Float64, queue_size=1)
    right_pub = rospy.Publisher('/robot_do_an/right_wheel_controller/command', Float64, queue_size=1)
    left_pub.publish(data_left)
    right_pub.publish(data_right)

if __name__ == '__main__':
    listener()
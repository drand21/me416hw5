#!/usr/bin/env python
'''
question 1.1 from hw5
'''
from curses import KEY_PPAGE
from importlib.machinery import WindowsRegistryFinder
from msilib.schema import PublishComponent
import rospy
import numpy as np
import motor_command_model as mcm
import controller as cr
import image_processing as ip
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

def callback(rosmsg):
    global LIN_SPEED, KP, KD, KI, pub
    LIN_SPEED = 0
    KP = 0
    KD = 0
    KI = 0
    stamped_msg_register = mcm.StampedMsgRegister()
    temppoint = rosmsg
    temp1 = temppoint.point.x
    temp2 = 160
    error_signal = temp2 - temp1

    temp3 = stamped_msg_register.previous_stamp()
    temp4 = temppoint.header.stamp
    time_delay = temp4-temp3

    msg = Twist()
    msg.linear.x = LIN_SPEED
    msg.angular.z = cr.pid.proportional(error_signal) + cr.derivative(error_signal, time_delay) + cr.integral(error_signal, time_delay)
    
    pub.publish(msg)


def main():
    global LIN_SPEED, KP, KD, KI, pub
    LIN_SPEED = 0
    KP = 0
    KD = 0
    KI = 0
    rospy.init_node('main')
    rospy.Subscriber('/image/centroid', PointStamped, callback=callback, queue_size=1, buff_size=2**18)
    pub = rospy.Publisher('robot_twist', Twist, queue_size=10)
    pid = cr.PID(KP,KD,KI)
    stamped_msg_register = mcm.StampedMsgRegister()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()

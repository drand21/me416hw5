"""Functions for modeling ROSBot"""

from turtle import right
from math import cos, sin
import numpy as np
from math import cos, sin
import rospy as rp


def model_parameters():
    """Returns two constant model parameters"""
    k_local = 1.0
    d_local = 0.5
    return k_local, d_local


def system_matrix(theta):
    """Returns a numpy array with the A(theta) matrix for a differential drive robot"""
    k, d = model_parameters()
    A = (k / 2.) * np.array([[cos(theta), cos(theta)],
                             [sin(theta), sin(theta)], [(-1. / d), (1. / d)]])
    return A


def system_field(z, u):
    """Computes the field at a given state for the dynamical model"""
    return dot_z


def twist_to_speeds(speed_linear, speed_angular):
    right = float()
    left = float()
    k, d = model_parameters()
    right = ((d / k) * speed_angular) + ((1 / k) * speed_linear)
    left = ((1 / k) * (speed_linear)) - ((d / k) * speed_angular)
    if -1.0 <= right <= 1.0 and -1.0 <= left <= 1.0:
        return left, right


def euler_step(z, u, stepSize):
    """Integrates the dynamical model for one time step using Euler's method"""
    theta = z[2, 0]
    A = system_matrix(theta)
    z_step = z + stepSize * A.dot(u)
    return z_step


class KeysToVelocities(object):
    def __init__(self):
        self.speed_angular = 0.0
        self.speed_linear = 0.0
        self.SPEED_DELTA = 0.2
        self.text_description = ""

    def update_speeds(self, key):
        if key == 'w' and self.speed_linear <= 0.8 or key == 'W' and self.speed_linear <= 0.8:
            self.speed_linear = self.speed_linear + self.SPEED_DELTA
            self.text_description = "Increase Linear Speed"
        elif key == 's' and self.speed_linear >= -0.8 or key == 'S' and self.speed_linear >= -0.8:
            self.speed_linear = self.speed_linear - self.SPEED_DELTA
            self.text_description = "Decrease Linear Speed"
        elif key == 'a' and self.speed_angular <= 0.8 or key == 'A' and self.speed_angular <= 0.8:
            self.speed_angular = self.speed_angular + self.SPEED_DELTA
            self.text_description = "Increase Angular Speed"
        elif key == 'd' and self.speed_angular >= -0.8 or key == 'D' and self.speed_angular >= -0.8:
            self.speed_angular = self.speed_angular - self.SPEED_DELTA
            self.text_description = "Decrease Angular Speed"
        elif key == 'z' or key == 'Z':
            self.speed_linear = 0.0
            self.text_description = "Set Linear Speed to Zero"
        elif key == 'c' or key == 'C':
            self.speed_angular = 0.0
            self.text_description = "Set Angular Speed to Zero"
        elif key == 'x' or key == 'X':
            self.speed_linear = 0.0
            self.speed_angular = 0.0
            self.text_description = "Set Linear and Angular Speed to Zero"
        return self.speed_linear, self.speed_angular, self.text_description


class StampedMsgRegister(object):
    def __init__(self):
        self.msg_previous = None

    def replace_and_compute_delay(self, msg):
        if self.msg_previous == None:
            time_delay = None
            msg_previous = self.msg_previous
            self.msg_previous = msg
        elif self.msg_previous != None:
            time_delay = msg.header.stamp.to_sec(
            ) - self.msg_previous.header.stamp.to_sec()
            msg_previous = self.msg_previous
            self.msg_previous = msg
        return time_delay, msg_previous

    def previous_stamp(self):
        if self.msg_previous == None:
            stamp = None
        else:
            stamp = self.msg_previous.header.stamp
        return stamp

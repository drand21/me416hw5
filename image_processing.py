#!/usr/bin/env python
""" A first template for a PID controller """

class PID(object):
    """ Computes proportional, derivative, and integral terms for a PID controller """

    def __init__(self, kp=1.0, kd=1.0, ki=1.0):
        """Initializes gains and internal state (previous error and integral error)"""
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.error_signal_previous = None
        self.error_signal_integral = 0

    def proportional(self, error_signal):
        """ Compute proportional term (with gain) """
        temp1 = self.kp*error_signal
        f_b = temp1 * -1
        return f_b

    def integral(self, error_signal, time_delay):
        """ Compute integral term (with gain) """
        temp2 = error_signal * time_delay
        self.error_signal_integral = self.error_signal_integral + temp2
        temp3 = self.ki * self.error_signal_integral
        f_i = temp3 * -1
        return f_i

    def derivative(self, error_signal, time_delay):
        """ Compute derivative term (with gain) """
        #TODO: This is a stub. Write your code here.
        if self.error_signal_previous==None:
            f_d = 0
        else:
            temp4 = error_signal - self.error_signal_previous
            temp4 = temp4 / time_delay
            temp4 = temp4 * self.kd
            f_d = temp4 * -1
        self.error_signal_previous = error_signal
        return f_d


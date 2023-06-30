# Kalman Filter Implementation for MPU-6050 6DOF IMU
#
# Author: Philip Salmony [pms67@cam.ac.uk]
# Date: 3 August 2018

import numpy as np
from math import sin, cos, tan




class Filter(object):

    def __init__(self):
        self.t = None
        self.C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
        self.P = np.eye(4)
        self.Q = np.eye(4)
        self.R = np.eye(2)

        self.state_estimate = np.array([[0], [0], [0], [0]])

        self.phi_hat = 0.0
        self.theta_hat = 0.0

        self.phi_offset = 0.0
        self.theta_offset = 0.0

        self.aFiltered = [0.0, 0.0, -1.0]
        self.acc = [0.0, 0.0, 0.0]
        self.filterN = 4

        self.p = 0
        self.q = 0
        self.r = 0

    def calculateOffsets(self, imu, N): #TODO add gyro offset
        for i in range(N):
            phi_acc, theta_acc, t = imu.getAccAngles()
            self.phi_offset += phi_acc
            self.theta_offset += theta_acc
        self.phi_offset = float(self.phi_offset) / float(N)
        self.theta_offset = float(self.theta_offset) / float(N)

    def run(self, t, phi_acc, theta_acc, p, q, r):
        phi_acc -= self.phi_offset
        theta_acc -= self.theta_offset

        # Gey gyro measurements and calculate Euler angle derivatives
        phi_dot = self.p + sin(self.phi_hat) * tan(self.theta_hat) * self.q + cos(self.phi_hat) * tan(self.theta_hat) * self.r
        theta_dot = cos(self.phi_hat) * self.q - sin(self.phi_hat) * self.r
        # p, q, r, t = imu.get_gyro()

        #TODO add bias
        self.p = p
        self.q = q
        self.r = r

        if self.t is None:
            dt = 0
        else:
            dt = t - self.t
        self.t = t

        # Kalman filter
        A = np.array([[1, -dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, -dt], [0, 0, 0, 1]])
        B = np.array([[dt, 0], [0, 0], [0, dt], [0, 0]])

        gyro_input = np.array([[phi_dot], [theta_dot]])
        state_estimate = A.dot(self.state_estimate) + B.dot(gyro_input)
        self.P = A.dot(self.P.dot(np.transpose(A))) + self.Q

        measurement = np.array([[phi_acc], [theta_acc]])
        y_tilde = measurement - self.C.dot(state_estimate)
        S = self.R + self.C.dot(self.P.dot(np.transpose(self.C)))
        K = self.P.dot(np.transpose(self.C).dot(np.linalg.inv(S)))
        state_estimate = state_estimate + K.dot(y_tilde)
        self.P = (np.eye(4) - K.dot(self.C)).dot(self.P)

        self.phi_hat = state_estimate[0]
        self.theta_hat = state_estimate[2]

        return self.phi_hat, self.theta_hat


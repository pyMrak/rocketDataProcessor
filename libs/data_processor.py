from math import pi, cos, sin, asin, atan2, sqrt
from libs import BMP085
from pandas import DataFrame
from numpy import zeros

class Data_Processor(object):

    def __init__(self, acc_scale, gyro_scale, press_range, p_calib_param, length, acc_filter_N, gyro_filter_N):
        self.p0 = 1.01325e5
        self.t_scale = 1000
        self.acc_scale = acc_scale
        self.gyro_scale = gyro_scale
        self.press_range = press_range
        self.p_calib_param = p_calib_param

        self.accFiltered = [0, 0, acc_scale]
        self.acc_filter_N = acc_filter_N

        self.gyroFiltered = [0, 0, 0]
        self.gyro_filter_N = gyro_filter_N

        self.h = -1
        # self.T = 0
        self.h_k = 0
        self.t_h = 0
        self.t_p = 0
        # self.T_k = 0
        # self.t_T = 0
        self.v_a = 0
        self.h_a = 0
        self.dt = 0

        self.current_idx = -1
        data = zeros((length-1, 35))
        self.dataFrame = DataFrame(data,
                                   columns=["t_r", "p_r", "T_r", "ax_r", "ay_r", "az_r", "gx_r", "gy_r", "gz_r",
                                            "t_f", "ax_f", "ay_f", "az_f", "gx_f", "gy_f", "gz_f",
                                            "t_p", "p_p", "ax_p", "ay_p", "az_p", "gx_p", "gy_p", "gz_p",
                                            "phi_a", "theta_a", "phi_g", "theta_g", "phi_f", "theta_f",
                                            "a_h", "h_p", "h_a", "h_f", "h_r"],
                                   )

    def filter_data(self, t, p, T, ax, ay, az, gy, gx, gz):
        self.current_idx += 1
        self.save_raw(t, p, T, ax, ay, az, gy, gx, gz)

        self.accFiltered[0] = (self.accFiltered[0] * (self.acc_filter_N - 1) + ax) / self.acc_filter_N
        self.accFiltered[1] = (self.accFiltered[1] * (self.acc_filter_N - 1) + ay) / self.acc_filter_N
        self.accFiltered[2] = (self.accFiltered[2] * (self.acc_filter_N - 1) + az) / self.acc_filter_N

        self.gyroFiltered[0] = (self.gyroFiltered[0] * (self.gyro_filter_N - 1) + gx) / self.gyro_filter_N
        self.gyroFiltered[1] = (self.gyroFiltered[1] * (self.gyro_filter_N - 1) + gy) / self.gyro_filter_N
        self.gyroFiltered[2] = (self.gyroFiltered[2] * (self.gyro_filter_N - 1) + gz) / self.gyro_filter_N

        ax = self.accFiltered[0]
        ay = self.accFiltered[1]
        az = self.accFiltered[2]
        gx = self.gyroFiltered[0]
        gy = self.gyroFiltered[1]
        gz = self.gyroFiltered[2]

        if p > 0:
            pr = BMP085.getPressure(p, T, self.press_range, self.p_calib_param)
            h = 44330 * (1.0 - pow(pr / self.p0, 0.190295))
        else:
            h = -1
        self.save_h_raw(h)

        if self.t_h == 0:
            self.t_h = t - 1
        if self.t_p == 0:
            self.t_p = t - 1
        if t == self.t_p:
            t += 0.5
        self.dt = t - self.t_p
        self.t_p = t
        if -1 != h:
            if t == self.t_h:
                t += 0.5
            dt = t - self.t_h
            if self.h == -1:
                self.h = h
            if self.dt != 0:
                self.h_k = (h - self.h) / dt
            self.h = h
            self.t_h = t
        else:
            h = self.h + self.h_k*(t-self.t_h)

        # if 32767 != T:
        #     # dt = t - self.t_T
        #     if self.T == 0:
        #         self.T = T
        #     if self.dt != 0:
        #         self.T_k = (T - self.T) / self.dt
        #     self.T = T
        #     self.t_T = t
        #     T = self.T
        # else:
        #     T = int(self.T + self.T_k*(t-self.t_T))
        self.save_filtered(t, h, ax, ay, az, gy, gx, gz)
        return t, h, ax, ay, az, gy, gx, gz


    def process_raw_data(self, t, h, ax, ay, az, gy, gx, gz):
        t = t / self.t_scale
        ax = ax / self.acc_scale
        ay = ay / self.acc_scale
        az = az / self.acc_scale
        gx = gx / self.gyro_scale * pi / 180
        gy = gy / self.gyro_scale * pi / 180
        gz = gz / self.gyro_scale * pi / 180
        self.save_processed(t, 0, h, ax, ay, az, gy, gx, gz)  # TODO add processed p
        return t, h, ax, ay, az, gx, gy, gz

    def post_filter_data(self, t, h, ax, ay, az, gy, gx, gz):
        return t, h, ax, ay, az, gy, gx, gz


    def getAccAngles(self, ax, ay, az):
        if sqrt(ax**2 + ay**2 + az**2) < 10:  # if we are not accelerating
            phi = atan2(ay, sqrt(ax ** 2.0 + az ** 2.0))
            theta = atan2(-ax, sqrt(ay ** 2.0 + az ** 2.0))
        else:  # if we are accelerating
            if abs(ay) < 9.81:
                phi = asin(ay/9.81)*ay/abs(ay)
            else:
                phi = pi / 2
            if abs(ax) < 9.81:
                theta = asin(ax/9.81)*ax/abs(ax)
            else:
                theta = pi / 2
        self.save_accAngles(phi, theta)
        return phi, theta

    def getAccZ(self, ax, ay, az, phi, theta):
        accZ = (az * cos(phi) * cos(theta) + ay * sin(phi) + ax * sin(theta) + 1) * 9.81
        if self.h_a == 0:
            self.h_a = self.dataFrame["h_p"][self.current_idx]
        if self.dataFrame["t_r"][self.current_idx] > 120903:
            b = 1
        self.v_a -= self.dt/1000 * accZ
        self.h_a += (self.dt/1000) * self.v_a - accZ * (self.dt/1000) ** 2 / 2
        self.save_accZ(accZ)
        self.save_h_acc(self.h_a)
        return accZ

    def save_raw(self, t, p, T, ax, ay, az, gy, gx, gz):
        self.dataFrame.loc[self.current_idx, "t_r"] = t
        self.dataFrame.loc[self.current_idx, "p_r"] = p
        self.dataFrame.loc[self.current_idx, "T_r"] = T
        self.dataFrame.loc[self.current_idx, "ax_r"] = ax
        self.dataFrame.loc[self.current_idx, "ay_r"] = ay
        self.dataFrame.loc[self.current_idx, "az_r"] = az
        self.dataFrame.loc[self.current_idx, "gx_r"] = gx
        self.dataFrame.loc[self.current_idx, "gy_r"] = gy
        self.dataFrame.loc[self.current_idx, "gz_r"] = gz

    def save_filtered(self, t, h, ax, ay, az, gy, gx, gz):
        self.dataFrame.loc[self.current_idx, "t_f"] = t
        self.dataFrame.loc[self.current_idx, "h_f"] = h
        self.dataFrame.loc[self.current_idx, "ax_f"] = ax
        self.dataFrame.loc[self.current_idx, "ay_f"] = ay
        self.dataFrame.loc[self.current_idx, "az_f"] = az
        self.dataFrame.loc[self.current_idx, "gx_f"] = gx
        self.dataFrame.loc[self.current_idx, "gy_f"] = gy
        self.dataFrame.loc[self.current_idx, "gz_f"] = gz

    def save_processed(self, t, p, h, ax, ay, az, gy, gx, gz):
        self.dataFrame.loc[self.current_idx, "t_p"] = t
        self.dataFrame.loc[self.current_idx, "p_p"] = p
        self.dataFrame.loc[self.current_idx, "h_p"] = h
        self.dataFrame.loc[self.current_idx, "ax_p"] = ax
        self.dataFrame.loc[self.current_idx, "ay_p"] = ay
        self.dataFrame.loc[self.current_idx, "az_p"] = az
        self.dataFrame.loc[self.current_idx, "gx_p"] = gx
        self.dataFrame.loc[self.current_idx, "gy_p"] = gy
        self.dataFrame.loc[self.current_idx, "gz_p"] = gz

    def save_accAngles(self, phi, theta):
        self.dataFrame.loc[self.current_idx, "phi_a"] = phi
        self.dataFrame.loc[self.current_idx, "theta_a"] = theta

    def save_gyroAngles(self, phi, theta):
        self.dataFrame.loc[self.current_idx, "phi_g"] = phi
        self.dataFrame.loc[self.current_idx, "theta_g"] = theta

    def save_filtAngles(self, phi, theta):
        self.dataFrame.loc[self.current_idx, "phi_f"] = phi
        self.dataFrame.loc[self.current_idx, "theta_f"] = theta

    def save_accZ(self, a):
        self.dataFrame.loc[self.current_idx, "a_h"] = a

    def save_h_acc(self, h):
        self.dataFrame.loc[self.current_idx, "h_a"] = h

    def save_h_filtered(self, h):
        self.dataFrame.loc[self.current_idx, "h_f"] = h

    def save_h_raw(self, h):
        self.dataFrame.loc[self.current_idx, "h_r"] = h


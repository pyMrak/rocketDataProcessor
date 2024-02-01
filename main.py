from libs.imu import *
from libs import BMP085
from libs.data_processor import Data_Processor
from kalman_filter.kalman6dof import Filter
from math import pi, cos, sin, sqrt
from filterpy.common import Q_discrete_white_noise
import numpy as np
from filterpy.kalman import KalmanFilter
from matplotlib import pyplot as plt

rho = 1.293
Af = 9.535e-4
As = 0.326
cd = 3.5
DC = 0.5*rho*cd*Af
dt = 0.01
std_a = 0.0374
std_h = 0.3281 #STD 0.2722 #ULTRAHIGH
std = 0.04
var = std * std




def main(file, pressure_overs):
    t_prev = 0
    dt = 0.01
    imu = IMU_Simulated(file)
    dp = Data_Processor(2048, 131.072, pressure_overs, imu.calibParam, imu.dataLen, 4, 2)
    kf_xy = Filter()
    kf_z = KalmanFilter(dim_x=3, dim_z=2)
    kf_z.H = np.array([[1, 0, 0], [0, 0, 1]])
    kf_z.F = np.array([[1, dt, dt * dt * 0.5], [0, 1, dt], [0, 0, 1]])
    kf_z.P = np.array([[std_h * std_h, 0, 0], [0, 1, 0], [0, 0, std_a * std_a]])
    dataAvailable = True
    while dataAvailable:
        t, P, T, ax, ay, az, p, q, r, fs, end = imu.getNewData()
        dataAvailable = not end
        t, h, ax, ay, az, p, q, r = dp.filter_data(t, P, T, ax, ay, az, p, q, r)
        t, h, ax, ay, az, p, q, r = dp.process_raw_data(t, h, ax, ay, az, p, q, r)
        dt = t - t_prev
        t_prev = t
        phiAcc, thetaAcc = dp.getAccAngles(ax, ay, az)
        phi, theta = kf_xy.run(t, phiAcc, thetaAcc, p, q, r)
        dp.save_filtAngles(phi, theta)
        acc_Z = dp.getAccZ(ax, ay, az, phi, theta)
        kf_z.F = np.array([[1, dt, dt * dt * 0.5], [0, 1, dt], [0, 0, 1]])
        kf_z.Q = Q_discrete_white_noise(dim=3, dt=dt, var=var)
        kf_z.predict()
        kf_z.update(np.array([[h], [-acc_Z]]))
        dp.save_h_filtered(kf_z.x[0])
    return dp



if __name__ == "__main__":
    data = main("flights/LOG00226.TXT", BMP085.BMP085_STANDARD) #"7th_flight.TXT"
    print(data.dataFrame["t_p"])
    # plt.plot(data.dataFrame["t_p"], data.dataFrame["p_r"])
    plt.plot(data.dataFrame["t_p"], data.dataFrame["h_p"])

    plt.plot(data.dataFrame["t_p"], data.dataFrame["h_a"])
    # plt.plot(data.dataFrame["t_p"], data.dataFrame["a_h"])
    plt.plot(data.dataFrame["t_p"], data.dataFrame["h_f"])
    plt.grid()
    # plt.xlim((120, 170))
    # plt.ylim((190, 380))
    plt.show()
    # imu = IMU_Simulated("7th_flight.TXT", 2048, 131.072)#"F:/LOG00226.TXT, 2048, 131.072)  #"F:/LOG00217.TXT" "secondFlight.csv", 2048, 131.072)  # "firstFlight.csv", 240, 131.072)
    # k = Filter()
    # dataAvailable = True
    # zaArr, zpArr, vzArr, accZarr, phiArr, thetaArr, fsArr, tArr, cdArr, tcdArr, vcdArr, pArr, TArr = [], [], [], [], [], [], [], [], [], [], [], [], []
    # axArr, ayArr, azArr, kf_h, kf_v = [], [], [], [], []
    # tPrev = None
    # za = 0
    # zp = 0
    # zpPrev = 0
    # v = 0
    # dt = 0
    # accFiltered = [0.0, 0.0, -1.0]
    # filterN = 2
    #
    #
    #     accFiltered[0] = (accFiltered[0] * (filterN - 1) + ax) / filterN
    #     accFiltered[1] = (accFiltered[1] * (filterN - 1) + ay) / filterN
    #     accFiltered[2] = (accFiltered[2] * (filterN - 1) + az) / filterN
    #     #zpArr.append(zp)
    #     axArr.append(accFiltered[0])
    #     ayArr.append(accFiltered[1])
    #     azArr.append(accFiltered[2])
    #     if(P):
    #         phiAcc, thetaAcc = imu.getAccAngles(accFiltered[0], accFiltered[1], accFiltered[2])
    #         # phi, theta = k.get_acc_angles(ax, ay, az)
    #         # dataAvailable = not end
    #         phi, theta = k.run(t, phiAcc, thetaAcc, p, q, r)
    #         #alpha = phi*cos()
    #         phiArr.append(phi/pi*360)
    #         thetaArr.append(theta/pi*360)
    #         accZ = (az*cos(phi)*cos(theta)+ay*sin(phi)+ax*sin(theta) + 1)*9.81
    #
    #         if tPrev is not None:
    #             dt = k.t/1000 - tPrev
    #             if fs:
    #
    #                 zpprev = zp
    #             if dt > 0:
    #                 vp = (13 * vp + (zp - zpPrev)/dt)/14
    #
    #         else:
    #             kf.x[0] = zp
    #             za = zp
    #         kf_h.append(kf.x[0]-203)
    #         kf_v.append(kf.x[1])
    #
    #
    #
    #         v -= dt*accZ
    #         za += dt*v - accZ*dt**2/2
    #         TArr.append(T)
    #         pArr.append(P)
    #         zpArr.append(zp-203)
    #         tPrev = k.t/1000
    #         accZarr.append(accZ+90)
    #         tArr.append(k.t/1000)
    #         zaArr.append(za-203)
    #         vzArr.append(v)
    #         fsArr.append(fs*30)
    #
    #         dataAvailable = not end
    #
    #         if k.t > 0:#187375:# and k.t < 190500:
    #             Fd = abs(accZ*0.145)
    #             cd = sqrt(Fd/DC)
    #             cdArr.append(cd)
    #             tcdArr.append(k.t/1000)
    #             vcdArr.append(v)
    #
    # # plt.plot(tArr[100:829], axArr[100:829])
    # # plt.plot(tArr[100:829], ayArr[100:829])
    # # plt.plot(tArr[100:829], azArr[100:829])
    # # plt.plot(tArr, phiArr)
    # # plt.plot(tArr, thetaArr)
    # # plt.plot(tArr, accZarr)
    # #plt.plot(tArr[304:], accZarr[304:])
    # # plt.plot(tArr, fsArr)
    # plt.plot(tArr, zaArr)
    # plt.plot(tArr, zpArr)
    # plt.plot(tArr, kf_h)
    # #plt.plot(tArr, kf_v)
    # print(zpArr)
    # # plt.plot(tcdArr, cdArr)
    # # plt.plot(tcdArr, vcdArr)
    # plt.grid()
    # #plt.xlim((120, 130))
    # plt.ylim((-10, 200))
    # plt.show()

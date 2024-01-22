from libs.imu import *
from libs import BMP085
from kalman_filter.kalman6dof import Filter
from math import pi, cos, sin, sqrt
from filterpy.common import Q_discrete_white_noise
import numpy as np
from filterpy.kalman import KalmanFilter

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


kf = KalmanFilter(dim_x=3, dim_z=2)
kf.H = np.array([[1, 0, 0], [0, 0, 1]])
kf.F = np.array([[1, dt, dt * dt * 0.5], [0, 1, dt], [0, 0, 1]])
kf.P = np.array([[std_h * std_h, 0, 0], [0, 1, 0], [0, 0, std_a * std_a]])

def main(file, pressure_acc):
    imu = IMU_Simulated(file, 2048, 131.072)


if __name__ == "__main__":
    from matplotlib import pyplot as plt
    imu = IMU_Simulated("7th_flight.TXT", 2048, 131.072)#"F:/LOG00226.TXT, 2048, 131.072)  #"F:/LOG00217.TXT" "secondFlight.csv", 2048, 131.072)  # "firstFlight.csv", 240, 131.072)
    k = Filter()
    dataAvailable = True
    zaArr, zpArr, vzArr, accZarr, phiArr, thetaArr, fsArr, tArr, cdArr, tcdArr, vcdArr, pArr, TArr = [], [], [], [], [], [], [], [], [], [], [], [], []
    axArr, ayArr, azArr, kf_h, kf_v = [], [], [], [], []
    tPrev = None
    za = 0
    zp = 0
    zpPrev = 0
    v = 0
    dt = 0
    accFiltered = [0.0, 0.0, -1.0]
    filterN = 2
    p0 = 1.01325e5
    vp = 0
    zpprev = 0
    while dataAvailable:

        t, P, T, ax, ay, az, p, q, r, fs, end = imu.getNewData()
        pressure = BMP085.getPressure(P, T, imu.calibParam)
        zpPrev = zp
        zp = 44330*(1.0 - pow(pressure/p0, 0.190295))

        accFiltered[0] = (accFiltered[0] * (filterN - 1) + ax) / filterN
        accFiltered[1] = (accFiltered[1] * (filterN - 1) + ay) / filterN
        accFiltered[2] = (accFiltered[2] * (filterN - 1) + az) / filterN
        #zpArr.append(zp)
        axArr.append(accFiltered[0])
        ayArr.append(accFiltered[1])
        azArr.append(accFiltered[2])
        if(P):
            phiAcc, thetaAcc = imu.getAccAngles(accFiltered[0], accFiltered[1], accFiltered[2])
            # phi, theta = k.get_acc_angles(ax, ay, az)
            # dataAvailable = not end
            phi, theta = k.run(t, phiAcc, thetaAcc, p, q, r)
            #alpha = phi*cos()
            phiArr.append(phi/pi*360)
            thetaArr.append(theta/pi*360)
            accZ = (az*cos(phi)*cos(theta)+ay*sin(phi)+ax*sin(theta) + 1)*9.81

            if tPrev is not None:
                dt = k.t/1000 - tPrev
                if fs:
                    kf.F = np.array([[1, dt, dt * dt * 0.5], [0, 1, dt], [0, 0, 1]])
                    kf.Q = Q_discrete_white_noise(dim=3, dt=dt, var=var)
                    kf.predict()
                    kf.update(np.array([[zp], [-accZ]]))
                    zpprev = zp
                if dt > 0:
                    vp = (13 * vp + (zp - zpPrev)/dt)/14

            else:
                kf.x[0] = zp
                za = zp
            kf_h.append(kf.x[0]-203)
            kf_v.append(kf.x[1])



            v -= dt*accZ
            za += dt*v - accZ*dt**2/2
            TArr.append(T)
            pArr.append(P)
            zpArr.append(zp-203)
            tPrev = k.t/1000
            accZarr.append(accZ+90)
            tArr.append(k.t/1000)
            zaArr.append(za-203)
            vzArr.append(v)
            fsArr.append(fs*30)

            dataAvailable = not end

            if k.t > 0:#187375:# and k.t < 190500:
                Fd = abs(accZ*0.145)
                cd = sqrt(Fd/DC)
                cdArr.append(cd)
                tcdArr.append(k.t/1000)
                vcdArr.append(v)

    # plt.plot(tArr[100:829], axArr[100:829])
    # plt.plot(tArr[100:829], ayArr[100:829])
    # plt.plot(tArr[100:829], azArr[100:829])
    # plt.plot(tArr, phiArr)
    # plt.plot(tArr, thetaArr)
    # plt.plot(tArr, accZarr)
    #plt.plot(tArr[304:], accZarr[304:])
    # plt.plot(tArr, fsArr)
    plt.plot(tArr, zaArr)
    plt.plot(tArr, zpArr)
    plt.plot(tArr, kf_h)
    #plt.plot(tArr, kf_v)
    print(zpArr)
    # plt.plot(tcdArr, cdArr)
    # plt.plot(tcdArr, vcdArr)
    plt.grid()
    #plt.xlim((120, 130))
    plt.ylim((-10, 200))
    plt.show()

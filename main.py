from libs.imu import *
from libs import BMP085
from kalman_filter.kalman6dof import Filter
from math import pi, cos, sin, sqrt

rho = 1.293
Af = 9.535e-4
As = 0.326
cd = 3.5
DC = 0.5*rho*cd*Af

if __name__ == "__main__":
    from matplotlib import pyplot as plt
    imu = IMU_Simulated("firstFlight.csv", 240, 131.072)
    k = Filter()
    dataAvailable = True
    zaArr, zpArr, vzArr, accZarr, phiArr, thetaArr, fsArr, tArr, cdArr, tcdArr, vcdArr = [], [], [], [], [], [], [], [], [], [], []
    tPrev = None
    za = 0
    zp = 0
    zpPrev = 0
    v = 0
    dt = 0
    accFiltered = [0.0, 0.0, -1.0]
    filterN = 4
    p0 = 1.01325e5
    vp = 0
    while dataAvailable:

        t, P, T, ax, ay, az, p, q, r, fs, end = imu.getNewData()
        pressure = BMP085.getPressure(P, T)
        zpPrev = zp
        zp = 44330*(1.0 - pow(pressure/p0, 0.190295)) - 187.5


        #zpArr.append(zp)

        accFiltered[0] = (accFiltered[0] * (filterN - 1) + ax) / filterN
        accFiltered[1] = (accFiltered[1] * (filterN - 1) + ay) / filterN
        accFiltered[2] = (accFiltered[2] * (filterN - 1) + az) / filterN

        phiAcc, thetaAcc = imu.getAccAngles(accFiltered[0], accFiltered[1], accFiltered[2])
        # phi, theta = k.get_acc_angles(ax, ay, az)
        # dataAvailable = not end
        phi, theta = k.run(t, phiAcc, thetaAcc, p, q, r)
        alpha = phi*cos()
        phiArr.append(phi/pi*360)
        thetaArr.append(theta/pi*360)
        accZ = az*cos(phi)*cos(theta)+ay*sin(phi)+ax*sin(theta)
        if tPrev is not None:
            dt = k.t/1000 - tPrev
            if dt > 0:
                vp = (13 * vp + (zp - zpPrev)/dt)/14
        v -= dt*(accZ+1)*9.81
        za += dt*v - accZ*dt**2/2
        zpArr.append(zp)
        tPrev = k.t/1000
        accZarr.append(accZ)
        tArr.append(k.t/1000)
        zaArr.append(za)
        vzArr.append(v)
        fsArr.append(fs)
        dataAvailable = not end

        if k.t > 0:#187375:# and k.t < 190500:
            Fd = abs((accZ+1)*9.81*0.145)
            cd = sqrt(Fd/DC)
            cdArr.append(cd)
            tcdArr.append(k.t/1000)
            vcdArr.append(v)



    # plt.plot(tArr, phiArr)
    # plt.plot(tArr, thetaArr)
    # plt.plot(tArr, accZarr)
    #plt.plot(tArr, fsArr)
    # plt.plot(tArr, zaArr)
    # plt.plot(tArr, zpArr)
    plt.plot(tcdArr, cdArr)
    plt.plot(tcdArr, vcdArr)
    plt.grid()
    plt.xlim((181, 195))
    plt.ylim((0, 40))
    plt.show()

import math
import json
from pandas import read_csv




class IMU_Simulated:

    def __init__(self, filePath, accScale, gyroScale):
        self.accScale = accScale
        self.gyroScale = gyroScale
        self.iterRows = None
        self.dataLen = 0
        self.dataIdx = 0
        self.p = 0
        self.T = 0
        self.p_k = 0
        self.t_p = 0
        self.T_k = 0
        self.t_T = 0


        if filePath[-4:].lower() == ".csv":
            self.data = read_csv(filePath, delimiter="\t")
            self.calibParam = {}
        elif filePath[-4:].lower() == ".txt":
            with open(filePath) as fd:
                headers = [next(fd) for i in range(2)]
                cols = headers[1].strip().replace(' ', '').split(',')
                calibParam = headers[0][7:].strip().replace('=', '":').replace(',', ',"').replace(' ', '')
                calibParam = '{"' + calibParam + '}'
                self.calibParam = json.loads(calibParam)
                self.data = read_csv(fd, sep=',',  names=cols)


    def getAccAngles(self, ax, ay, az):
        if abs(ay) < 9.81:
            phi = math.asin(ay/9.81)*ay/abs(ay)
        else:
            phi = math.pi / 2
        # phi = math.atan2(ay, math.sqrt(ax ** 2.0 + az ** 2.0))
        if abs(ax) < 9.81:
            theta = math.asin(ax/9.81)*ax/abs(ax)
        else:
            theta = math.pi / 2
        # theta = math.atan2(-ax, math.sqrt(ay ** 2.0 + az ** 2.0))
        return phi, theta

    def getNewData(self):
        if self.iterRows is None:
            self.iterRows = [row for row in self.data.iterrows()]
            self.dataLen = len(self.iterRows)
            self.dataIdx = 0
        index, dataRow = self.iterRows[self.dataIdx]
        end = False
        self.dataIdx += 1
        if self.dataIdx >= self.dataLen - 1:
            self.dataIdx = 0
            end = True
        t = dataRow["t"]
        if -1 != dataRow["p"]:
            dt = t - self.t_p
            if dt != 0:
                self.p_k = (dataRow["p"] - self.p) / dt
            self.p = dataRow["p"]
            self.t_p = t
            p = self.p
        else:
            p = int(self.p + self.p_k*(t-self.t_p))
        if 32767 != dataRow["T"]:
            dt = t - self.t_T
            if dt != 0:
                self.T_k = (dataRow["T"] - self.T) / dt
            self.T = dataRow["T"]
            self.t_T = t
            T = self.T
        else:
            T = int(self.T + self.T_k*(t-self.t_T))
        if end:
             t = t
        if not end:
            tNext = self.iterRows[self.dataIdx][1]["t"]
            if t > tNext:
                if self.dataIdx - 1:
                    tPrev = self.iterRows[self.dataIdx - 2][1]["t"]
                    t = (tPrev + tNext)/2
                else:
                    t = 2*tNext - self.iterRows[self.dataIdx + 1][1]["t"]


        return t, p, T, int(dataRow["ax"])/self.accScale,\
            int(dataRow["ay"])/self.accScale, int(dataRow["az"])/self.accScale,\
            int(dataRow["gx"])/self.gyroScale*math.pi/180,\
            int(dataRow["gy"])/self.gyroScale*math.pi/180,\
            int(dataRow["gz"])/self.gyroScale*math.pi/180, dataRow["fs"],end



if __name__ == "__main__":
    imu = IMU_Simulated("../kalman_filter/firstFlight.csv", 24.46, 131.072)
    dataAvailable = True
    while dataAvailable:
        t, ax,ay,az, gy,gy,gz, dataAvailable = imu.getNewData()
        dataAvailable = not(dataAvailable)
        print(t, ax)

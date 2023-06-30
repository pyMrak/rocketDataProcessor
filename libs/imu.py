import math
from pandas import read_csv



class IMU_Simulated:

    def __init__(self, filePath, accScale, gyroScale):
        self.accScale = accScale
        self.gyroScale = gyroScale
        self.iterRows = None
        self.dataLen = 0
        self.dataIdx = 0
        if filePath[-4:] == ".csv":
            self.data = read_csv(filePath, delimiter="\t")

    def getAccAngles(self, ax, ay, az):
        phi = math.atan2(ay, math.sqrt(ax ** 2.0 + az ** 2.0))
        theta = math.atan2(-ax, math.sqrt(ay ** 2.0 + az ** 2.0))
        return phi, theta

    def getNewData(self):
        if self.iterRows is None:
            self.iterRows = [row for row in self.data.iterrows()]
            self.dataLen = len(self.iterRows)
            self.dataIdx = 0
        index, dataRow = self.iterRows[self.dataIdx]
        end = False
        self.dataIdx += 1
        if self.dataIdx >= self.dataLen:
            self.dataIdx = 0
            end = True
        return dataRow["t"], dataRow["p"], dataRow["T"], int(dataRow["ax"])/self.accScale,\
            int(dataRow["ay"])/self.accScale, int(dataRow["az"])/self.accScale,\
            int(dataRow["gx"])/self.gyroScale*math.pi/180,\
            int(dataRow["gy"])/self.gyroScale*math.pi/180,\
            int(dataRow["gz"])/self.gyroScale*math.pi/180, end



if __name__ == "__main__":
    imu = IMU_Simulated("../kalman_filter/firstFlight.csv", 24.46, 131.072)
    dataAvailable = True
    while dataAvailable:
        t, ax,ay,az, gy,gy,gz, dataAvailable = imu.getNewData()
        dataAvailable = not(dataAvailable)
        print(t, ax)

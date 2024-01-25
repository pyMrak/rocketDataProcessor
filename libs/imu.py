import json
from pandas import read_csv




class IMU_Simulated:

    def __init__(self, filePath):
        self.iterRows = None
        self.dataLen = 0
        self.dataIdx = 0


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

        for i, row in self.data.iterrows():
            if row["p"] != -1:
                break
        self.data.drop(range(i), axis=0, inplace=True)
        self.iterRows = [row for row in self.data.iterrows()]
        self.dataLen = len(self.iterRows)

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


        return int(dataRow["t"]), int(dataRow["p"]), int(dataRow["T"]), int(dataRow["ax"]), int(dataRow["ay"]), int(dataRow["az"]),\
               int(dataRow["gx"]), int(dataRow["gy"]),  int(dataRow["gz"]), int(dataRow["fs"]), end



if __name__ == "__main__":
    imu = IMU_Simulated("../kalman_filter/firstFlight.csv", 24.46, 131.072)
    dataAvailable = True
    while dataAvailable:
        t, ax,ay,az, gy,gy,gz, dataAvailable = imu.getNewData()
        dataAvailable = not(dataAvailable)
        print(t, ax)

BMP085_ULTRALOWPOWER = 0 # Ultra low power mode
BMP085_STANDARD = 1      # Standard mode
BMP085_HIGHRES = 2       # High-res mode
BMP085_ULTRAHIGHRES = 3  # Ultra high-res mode

ac1 = 6631
ac2 = -954
ac3 = -14533
ac4 = 32773
ac5 = 25371
ac6 = 18958

b1 = 5498
b2 = 47

mb = -32768
mc = -11075
md = 2432

oversampling = BMP085_ULTRAHIGHRES

def computeB5(UT):
    X1 = (UT - ac6) * ac5 >> 15
    X2 = (mc << 11) / (X1 + md)
    return X1 + X2

def getPressure(Pr, Tr):

    B5 = computeB5(Tr)

    B6 = B5 - 4000
    X1 = (b2 * (int(B6 * B6) >> 12)) >> 11
    X2 = int(ac2 * B6) >> 11
    X3 = X1 + X2
    B3 = (((ac1 * 4 + X3) << oversampling) + 2) / 4

    X1 = int(ac3 * B6) >> 13
    X2 = (b1 * (int(B6 * B6) >> 12)) >> 16
    X3 = ((X1 + X2) + 2) >> 2
    B4 = (ac4 * (X3 + 32768)) >> 15
    B7 = (Pr - B3) * (50000 >> oversampling)

    if B7 < 0x80000000:
        p = int((B7 * 2) / B4)
    else:
        p = int((B7 / B4) * 2)
    X1 = (p >> 8) * (p >> 8)
    X1 = (X1 * 3038) >> 16
    X2 = (-7357 * p) >> 16

    p = p + ((X1 + X2 + 3791) >> 4)

    return p


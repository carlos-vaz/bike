import sys
from matplotlib import pyplot as plt
import time
import os
import serial


stabilizing = False
stabilizingWindow = 20

'''
arg = fmt_custom: reads lines in the following format:
(gX, gY, gZ) = (-135.00, -32.95, -244.32)

arg = fmt_arduino: reads lines in the following format:
Scaled. Acc (mg) [ -00012.70, -00041.02,  00987.79 ], Gyr (DPS) [ -00000.98,  00002.60,  00001.31 ], Mag (uT) [ -00007.65,  00031.20,  00012.60 ], Tmp (C) [  00029.13 ]

arg = fmt_cybergear
I (48543) imu_task: Acc: [ 72, -588, 16056 ] Gyr: [ -102, 49, 48 ] Mag: [ 0, 0, 0 ] Tmp: [ 3136 ]

arg = fmt_gxfilter
I (8019) imu_task: gxraw= -364		gxfiltered=-80.12
'''

def main(serialPort, baudRate, updatePeriod_ms, plotTimeWindow_s, outputMode, sensor):
    time.sleep(0.5)

    gyro = [[], [], []]

    plt.ion()
    ax = plt.axes()
    ax.plot([], 'r')
    ax.plot([], 'g')
    ax.plot([], 'b')

    bias = [0,0,0]

    customFmtLineToGyro =    lambda l: [float(n) for n in l.split('=')[1].strip()[1:-1].split(',')]
    arduinoFmtLineToGyro =   lambda l: [float(n) for n in l.split("Gyr (DPS)")[1].split(", Mag")[0].strip()[1:-1].split(",")]
    arduinoFmtLineToMag =    lambda l: [float(n) for n in l.split("Mag (uT)")[1].split(", Tmp")[0].strip()[1:-1].split(",")]
    arduinoFmtLineToAccel =  lambda l: [float(n) for n in l.split("Acc (mg)")[1].split(", Gyr")[0].strip()[1:-1].split(",")]
    cybergearFmtLineToGyro = lambda l: [float(n) for n in l.split("Gyr:")[1].split("Mag")[0].strip()[1:-1].split(",")]
    gxfilterFmtLineToGyro =  lambda l: [float(n) for n in l.split('gxraw=')[1].split('gxfiltered=')]

    with open('plot_log.txt', 'w') as f:
        with serial.Serial(serialPort, baudRate, timeout=1) as ser:
            while(1):
                l = ser.readline().decode('utf-8')
                f.write(l)
                if(not l):
                    # EOF, wait for more data...
                    time.sleep((updatePeriod_ms/1000))
                else:
                    try:
                        dataFields = 3
                        if(outputMode == "fmt_custom"):
                            gNew = customFmtLineToGyro(l)
                        elif(outputMode == "fmt_arduino"):
                            if(sensor == 'g'):
                                gNew = arduinoFmtLineToGyro(l)
                            elif(sensor == 'm'):
                                gNew = arduinoFmtLineToMag(l)
                            elif(sensor == 'a'):
                                gNew = arduinoFmtLineToAccel(l)
                        elif(outputMode == "fmt_cybergear"):
                            gNew = cybergearFmtLineToGyro(l)
                        elif(outputMode == "fmt_gxfilter"):
                            gNew = gxfilterFmtLineToGyro(l)
                            dataFields = 2
                        else:
                            gNew = [0,0,0]
                        [gyro[i].append(gNew[i] - bias[i]) for i in range(dataFields)]
                    except:
                        print(f"failed at line with contents: {l}")

                    if(stabilizing):
                        if(len(gyro[0]) == stabilizingWindow):
                            bias[0] = sum(gyro[0]) / stabilizingWindow
                        if(len(gyro[1]) == stabilizingWindow):
                            bias[1] = sum(gyro[1]) / stabilizingWindow
                        if(len(gyro[2]) == stabilizingWindow):
                            bias[2] = sum(gyro[2]) / stabilizingWindow
                    
                    ax.cla()
                    ax.set_xlim(0, int(plotTimeWindow_s / (updatePeriod_ms/1000)) + 1)
                    ax.plot(gyro[0][(int)(-1*plotTimeWindow_s / (updatePeriod_ms/1000)):], 'r')
                    ax.plot(gyro[1][(int)(-1*plotTimeWindow_s / (updatePeriod_ms/1000)):], 'g')
                    ax.plot(gyro[2][(int)(-1*plotTimeWindow_s / (updatePeriod_ms/1000)):], 'b')
                    plt.draw()
                    plt.pause((updatePeriod_ms/1000) * 0.1)

            plt.show()


if(__name__ == "__main__"):
    if(len(sys.argv) != 7):
        print("USAGE: plot_gyro.py [serial port] [baud_rate] [update period (ms)] [plot time window (s)] [output mode={fmt_arduino,fmt_custom}] [sensor={a,g,m}]")
        print("\te.g: plot_gyro.py /dev/ttyUSB0 115200 30 5 fmt_arduino g")
        exit(1)

    if((sys.argv[5] == "fmt_custom" or sys.argv[5] == "fmt_cybergear") and sys.argv[6] != "g"):
        print("fmt_{custom/cybergear} only support gyro (sensor = g)")
        exit(1)

    main(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), sys.argv[5], sys.argv[6])



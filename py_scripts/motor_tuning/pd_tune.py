import serial
import csv
import time
import os
import matplotlib.pyplot as plt
import math
import statistics

def send(kp, kd, speed, ser):
    data = bytes(str(math.floor(kp)).zfill(4) + str(math.floor(kd)).zfill(4) + str(speed).zfill(3), 'utf-8')
    ser.write(data)

def test(kp, kd, speed):
    ser = serial.Serial(com_port, baud_rate, timeout=None)
    ser.flushInput()

    send(kp, kd, speed, ser)

    first_row_written = False

    with open(os.path.join(os.path.dirname(__file__), 'out.csv'), 'w', newline='') as file:
        csv_writer = csv.writer(file)

        while (not ser.in_waiting):
            pass

        start_time = time.time_ns()

        while time.time_ns() - start_time < recording_time_s * 1000000:
            #read line of serial
            if ser.in_waiting:
                data = [[d.strip() for d in s.split(':')] for s in ser.readline().decode().split(',')]
                
                #write first row of labels (once)
                if not first_row_written:
                    csv_writer.writerow(['Timestamp'] + [s[0] for s in data])
                    first_row_written = True
                
                #write new row of data
                csv_writer.writerow([time.time_ns()//1000] + [s[1] for s in data])

    time.sleep(2)

def plot():
    with open(os.path.join(os.path.dirname(__file__), 'out.csv'), 'r', newline='') as file:
        csv_reader = csv.reader(file)

        x = []
        y = []

        skip_first = True
        for px, py in csv_reader:
            if skip_first:
                skip_first = False
            else:
                x.append(int(px))
                y.append(float(py))

        mean = statistics.fmean(y)
        y2 = [mean] * len(x)

        fig, ax = plt.subplots()
        ax.plot(x, y, linewidth = 2)
        ax.plot(x, y2, linewidth = 2)
        plt.ylim(-15, 15)
        plt.show()

if __name__ == "__main__":
    com_port = 'COM7'
    baud_rate = 115200
    recording_time_s = 1

    # EV3 MED
    # test(205, 0, 50)
    # test(164, 138, 5)
    # test(164, 138, 10)
    # test(164, 138, 15)
    # test(164, 138, 25)
    # test(164, 138, 50)
    # test(164, 138, 75)
    # test(164, 138, 100)
    # period 67ms

    # EV3 LARGE
    # test(265, 0, 50)
    # test(212, 403, 5)
    # test(212, 403, 10)
    # test(212, 403, 15)
    # test(212, 403, 25)
    # test(212, 403, 50)
    # test(212, 403, 75)
    # test(212, 403, 100)
    # period 152ms

    # NXT LARGE
    # test(235, 0, 50)
    # test(188, 250, 100)
    # test(188, 250, 75)
    # test(188, 250, 50)
    # test(188, 250, 25)
    # test(188, 250, 5)
    # period 111ms

    # JGA25-270 (test for custom)
    # test(500, 0, 50)
    # test(400, 480, 5)
    # period 110

    plot()
import serial
import struct
import time
from pynput.mouse import Button, Controller
import queue
from threading import Thread
#EKF
import ahrs
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
from ahrs.common.orientation import q2rpy
import numpy as np
#graceful ending
import signal
import sys

# Initialize mouse controller
mouse = Controller()
# Open the COM port
ser = serial.Serial('COM11', baudrate=115200, timeout=1)

START_BYTE = -128
gyr_alpha = 0.1
acc_alpha = 0.5

def StartStream():
    Aligned = False
    while Aligned == False:
        start_byte = ser.read(1)  # Read one byte
        if start_byte and int.from_bytes(start_byte, 'big', signed=True) == START_BYTE:  # Check for start byte (-128)
            Unpack("IMU_start",ser.read(28))
            next_start_byte = ser.read(1)  # Read one byte
            if next_start_byte and int.from_bytes(next_start_byte, 'big', signed=True) == START_BYTE:
                Unpack("IMU_start",ser.read(28))
                timestamp = 0
                timestamp_prev = 0
                while True:
                    Unpack("IMU",ser.read(29))

        else:
            #pass
            print("Byte not start byte -", int.from_bytes(start_byte, 'big', signed=True))
                #mouse.move(value1,value2)

def Unpack(mode, data):
    if mode == "IMU_start":
        if len(data) == 28:
            g_f32 = struct.unpack('fff', data[0:12])
            a_f32 = struct.unpack('fff', data[12:24])
            var1 = int.from_bytes(data[24:26], 'little', signed=False)
            var2 = int.from_bytes(data[26:28], 'little', signed=False)
            IMUDataQueue.put((a_f32, g_f32))
            AnalogDataQueue.put((var1,var2))
            #timestamp_prev = timestamp
            #timestamp = time.perf_counter()
            #print(f"t: {timestamp:.3f}, T: {(timestamp-timestamp_prev):.3f} g: {[f'{x:8.3g}' for x in g_f32]}, a: {[f'{x:8.3g}' for x in a_f32]}, X: {var1:5}, Y: {var2:5}")
            #print(f"g: {[f'{x:8.3g}' for x in g_f32]}, a: {[f'{x:8.3g}' for x in a_f32]}, X: {var1:5}, Y: {var2:5}")
        else:
            print("Data packet length invalid")
    elif mode == "IMU":
        if len(data) == 29:
            g_f32 = struct.unpack('fff', data[1:13])
            a_f32 = struct.unpack('fff', data[13:25])
            var1 = int.from_bytes(data[25:27], 'little', signed=False)
            var2 = int.from_bytes(data[27:29], 'little', signed=False)
            IMUDataQueue.put((a_f32, g_f32))
            AnalogDataQueue.put((var1,var2))
            #timestamp_prev = timestamp
            #timestamp = time.perf_counter()
            #print(f"t: {timestamp:.3f}, T: {(timestamp-timestamp_prev):.3f} g: {[f'{x:8.3g}' for x in g_f32]}, a: {[f'{x:8.3g}' for x in a_f32]}, X: {var1:5}, Y: {var2:5}")
            #print(f"g: {[f'{x:8.3g}' for x in g_f32]}, a: {[f'{x:8.3g}' for x in a_f32]}, X: {var1:5}, Y: {var2:5}")
        else:
            print("Data packet length invalid")
    else:
        print("Invalid mode entered")

def PerformEKF():
    flag = 1
    gyr_prev = []
    acc_prev = []
    gyr_cur = []
    acc_cur = []

    while True:
        task = IMUDataQueue.get()
        if task is not None:
            if flag == 1:
                acc, gyro = task
                acc_array = np.array([acc],dtype = float)
                gyro_array = np.array([gyro],dtype = float)
                acc_prev = acc_array
                gyr_prev = gyro_array
                flag = 0

            acc, gyro = task
            acc_array = np.array([acc],dtype = float)
            gyro_array = np.array([gyro],dtype = float)

            # Pre-filter
            acc_cur = (acc_alpha * acc_prev) + ((1 - acc_alpha) * (acc_array))
            gyr_cur = (gyr_alpha * gyr_prev) + ((1 - gyr_alpha) * (gyro_array))

            acc_prev = acc_cur
            gyr_prev = gyr_cur

            ekf = EKF(gyr_cur, acc_cur,frequency=19.0,frame='ENU')
            #ekf = EKF(gyr_cur, acc_cur,frequency=100.0)
            '''hor = int(ekf.Q[0,1]*5)
            ver = int(ekf.Q[0,2]*5)
            print(hor," ",ver)
            mouse.move(hor, ver)'''
            hor = int(((ekf.Q[0,1]+1)/2)*(1920-1))
            ver = mouse.position
            #mouse.position = (hor, ver[1])
            print(f"gyro: {[f'{x:8.3g}' for x in gyro]}, acc: {[f'{x:8.3g}' for x in acc]}")
            IMUDataQueue.task_done()
        else:
            print("IMUData slow")

def AnalyseAnalog():
    while True:
        task = AnalogDataQueue.get()
        if task is not None:
            X, Y = task
            if X <= 500:
                mouse.click(Button.left, 1)
            if Y <= 500:
                mouse.click(Button.right, 1)
            print(X, " ", Y)
            AnalogDataQueue.task_done()
        else:
            print("AnalogData slow")

def shutdown(signal, frame):
    print("\nShutting down threads (non)gracefully...")
    # Use signal to stop threads and cleanly exit
    sys.exit(0)

#KeyboardInterrupt (Ctrl+C)
signal.signal(signal.SIGINT, shutdown)

IMUDataQueue = queue.Queue()
AnalogDataQueue = queue.Queue()

StreamThread = Thread(target = StartStream, daemon = False)
EKFThread = Thread(target = PerformEKF, daemon = False)
AnalogThread = Thread(target = AnalyseAnalog, daemon = False)

StreamThread.start()
EKFThread.start()
AnalogThread.start()

while True:
    time.sleep(1)
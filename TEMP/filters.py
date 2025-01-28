import queue
import math
import time
import numpy as np
##### EKF libraries srt #####
import serial
from ahrs.filters import EKF
import ahrs
from ahrs.common.orientation import acc2q
from ahrs.common.orientation import q2rpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
##### EKF libraries end #####

#the most up-to-date pipeline structure
def no_filter(input_queue, output_queue, stop_event):
    while not stop_event.is_set():
        try:
            task = input_queue.get(timeout = 0.01)
            #do something
            output_queue.put(task)
            input_queue.task_done()
        except queue.Empty:
            continue
    print("no_filter_stopped")

def EKF(input_queue, output_queue, stop_event):
    ##### Setup your variables and flags str #####
    # predict_period = 0.01
    # update_period = 0.05
    # predict_clock = 0.0
    # update_clock = 0.0
    gyr_alpha = 0.05
    acc_alpha = 0.2
    
    gyr_prev = []
    acc_prev = []
    gyr_cur = []
    acc_cur = []

    Qold = np.zeros((1, 4))
    Qnew = np.zeros((1, 4))
    r2d = 57.2958
    your_data = []
    
    ##### Plotting var str #####
    # plot_count = 0
    # wplt = []
    # xplt = []
    # yplt = []
    # zplt = []
    # plt.axis([0, 9.8, -1, 1])
    # a = np.arange(0, 10, 0.05)
    # plt.grid()
    # plt.title("Euler Output of Extended Kalman Filter")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Rotation (°)")
    ##### Plotting var end #####

    ##### Setup your variables and flags end #####

    ##### Initialise EKF str #####
    while not gyr_prev:
        try:
            gyr_prev, acc_prev = input_queue.get(timeout = 0.01)
            Qold = acc2q(acc_prev[0]) # Get the first quaternion state array by converting accelerometer data into a quaternion
            #ekf = EKF(frequency=17.3,frame='ENU',q0=Qold,noises=[0.1**2, 0.15**4, 0.8**2]) # EKF init from old code
            ekf = EKF(frequency=90,frame='ENU',q0=Qold,noises=[0.1**2, 0.15**4, 0.8**2]) # Initialise EKF function. Noise = [Process covariance, Measurement covariance]
        except queue.Empty:
            continue
    ##### Initialise EKF end #####

    while not stop_event.is_set():
        try:
            gyro, acc = input_queue.get(timeout = 0.01)
            
            #Your code

            ##### Pre-filter str #####
            acc_cur = (acc_alpha * acc_prev) + ((1 - acc_alpha) * (acc))
            gyr_cur = (gyr_alpha * gyr_prev) + ((1 - gyr_alpha) * (gyro))

            acc_prev = acc_cur
            gyr_prev = gyr_cur
            ##### Pre-filter end #####

            ##### EKF update str #####
            Qnew = ekf.update(Qold, gyr_cur[0], acc_cur[0]) # Update EKF function. See: https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/ekf.py#L1336
            Qold = Qnew
            angles = q2rpy(Qnew)*r2d # Get euler angles from quaternions 
            your_data = [angles[1], angles[2]] # Send array [Roll, Pitch]
            # print(f"Q: {[f'{(100*x):8.2f}' for x in Qold]}, E: {[f'{(100*x):8.2f}' for x in angles]}") # Print quat and euler output of update
            ##### EKF update end #####

            ##### Graphing str #####
            # w = Qold[0] 
            # x = Qold[1] # Roll
            # y = Qold[2] # Pitch
            # z = Qold[3]
            # if(plot_count < 200):
            #     wplt.append(w)
            #     xplt.append(x)
            #     yplt.append(y)
            #     zplt.append(z)
            #     plot_count = 1+ plot_count
            # elif(plot_count == 200):
            #     plt.plot(a, wplt, '-', label = "w")
            #     plt.plot(a, xplt, '-', label = "x")
            #     plt.plot(a, yplt, '-', label = "y")
            #     plt.plot(a, zplt, '-', label = "z")
            #     plt.xticks(np.arange(0, 10, 0.5))
            #     plt.yticks(np.arange(-1, 1, 0.2))
            #     plt.legend()
            #     #plt.show()
            #     plot_count = 1 + plot_count
            ##### Graphing end #####

            outputqueue.put(your_data)
            inputqueue.task_done()
        except queue.Empty:
            continue
    print("EKF_stopped")


#####################################
# Complementary filter function
#####################################
def complementary_filter(input_queue, output_queue, stop_event):
    # Constants and initial values
    Pi = math.pi
    alpha = 0.97  # Filter coefficient
    dt = 0.01  # Time step

    # Initial estimated angles
    pitch_est = 0
    roll_est = 0

    while not stop_event.is_set():
        try:
            task = input_queue.get(timeout=0.01)  # Get data from the queue

            if task is not None:
            
                acc, gyro = task  # Unpack accelerometer and gyroscope data
                acc_array = np.array(acc, dtype=float)  # Convert to NumPy array for accelerometer data
                gyro_array = np.array(gyro, dtype=float)  # Convert to NumPy array for gyroscope data

                # Extract individual components from the arrays
                ax, ay, az = acc_array
                gx, gy, gz = gyro_array

                # Scale accelerometer values (assuming LSB of 16384 for accelerometer)
                ax /= 16384
                ay /= 16384
                az /= 16384

                # Gyroscope scaling (assuming scale of 262.4 LSB/deg/s from datasheet)
                gyroscale = 1 / 262.4
                pitch_g = gx * gyroscale * math.pi / 180  # Convert to radians
                roll_g = gy * gyroscale * math.pi / 180

                # Accelerometer angle estimation (in degrees)
                pitch_a = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / math.pi
                roll_a = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi

                # Apply complementary filter
                pitch_est = alpha * (pitch_est + pitch_g * dt) + (1 - alpha) * pitch_a
                roll_est = alpha * (roll_est + roll_g * dt) + (1 - alpha) * roll_a

                # Output filtered angles to the output queue
                output_queue.put((pitch_est, roll_est))

                # Mark task as done
                input_queue.task_done()
        except queue.Empty:
            continue

    print("complementary_filter_stopped yay!!!")

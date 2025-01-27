import queue
import math
import time
import numpy as np

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

#needs some work
def EKF(input_queue, output_queue, stop_event):
    flag = 1
    gyr_prev = []
    acc_prev = []
    gyr_cur = []
    acc_cur = []

    while not stop_event.is_set():
        task = input_queue.get()
        #Your code
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

            #Your code END

            inputqueue.task_done()
            outputqueue.put(your_data)

        else:
            print("IMUData slow")


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

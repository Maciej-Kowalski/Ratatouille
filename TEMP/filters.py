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

def EKF(input_queue, output_queue, stop_event):
    #setup your variables and flags

    while not stop_event.is_set():
        try:
            gyro, acc = input_queue.get(timeout = 0.01)
            
            #Your code

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

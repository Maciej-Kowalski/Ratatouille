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
from collections import deque
from scipy.signal import butter, filtfilt
#import matplotlib.pyplot as plt
#import matplotlib.animation as animation
#from matplotlib import style
##### EKF libraries end #####

import queue
import time
import numpy as np
from collections import deque
from scipy.signal import butter, filtfilt

# ---------------- Parameters ----------------
PACKET_SIZE = 100           # Samples per packet (must match serial_pipeline settings)
WINDOW_PACKETS = 20         # Number of packets to form the sliding window
SAMPLE_RATE = 16500         # Hz
LOWCUT = 7000.0             # Hz
HIGHCUT = 8000.0            # Hz
FILTER_ORDER = 5

# Click detection thresholds and cooldown
POSITIVE_THRESHOLD = 0.20
NEGATIVE_THRESHOLD = -0.20
click_cooldown = 0.18       # seconds

# ---------------- Helper Functions ----------------
def butter_bandpass_filter(data):
    nyquist = 0.5 * SAMPLE_RATE
    low = LOWCUT / nyquist
    high = HIGHCUT / nyquist
    b, a = butter(FILTER_ORDER, [low, high], btype='band')
    return filtfilt(b, a, data)

def detect_click_sliding_window(signal):
    return np.max(signal) > POSITIVE_THRESHOLD and np.min(signal) < NEGATIVE_THRESHOLD

# ---------------- New Filtering Function ----------------
def audio_filter_with_buffer(input_queue, output_queue, stop_event):
    """
    Reads audio packets from input_queue, buffers them in a sliding window,
    applies a bandpass filter, and performs click detection.
    
    Each packet is expected to be a NumPy array of length PACKET_SIZE.
    """
    packet_buffer = deque(maxlen=WINDOW_PACKETS)
    last_click_time = 0.0
    
    while not stop_event.is_set():
        try:
            # Retrieve next packet (blocking with a timeout)
            packet = input_queue.get(timeout=0.01)
            packet_buffer.append(packet)
            input_queue.task_done()
            
            # Process only if we have a full window
            if len(packet_buffer) < WINDOW_PACKETS:
                continue

            # Concatenate packets (oldest to newest)
            window = np.concatenate(list(packet_buffer))
            
            # Apply filtering
            filtered_window = butter_bandpass_filter(window)
            
            # Click detection on the filtered window
            current_time = time.time()
            if detect_click_sliding_window(filtered_window):
                if current_time - last_click_time > click_cooldown:
                    last_click_time = current_time
                    # Send a tuple indicating a click event along with the filtered data
                    output_queue.put(("click", filtered_window))
                    continue  # Skip sending normal data in this case
            
            # Otherwise, output the filtered data (e.g., for visualization or further processing)
            output_queue.put(("data", filtered_window))
            
        except queue.Empty:
            continue
        except Exception as e:
            print("Exception in audio_filter_with_buffer:", e)
            continue
    print("audio_filter_with_buffer stopped")
    

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

def EK_filter(input_queue, output_queue, stop_event):
    print("ek_filter run")
    ##### Setup your variables and flags str #####
    # predict_period = 0.01
    # update_period = 0.05
    # predict_clock = 0.0
    # update_clock = 0.0
    gyr_alpha = 0.1
    acc_alpha = 0.5
    salpha = 0.75 # Originally 0.65
    flg = 0
    
    gyr_prev = []
    acc_prev = []
    gyr_cur = []
    acc_cur = []

    Qold = np.zeros((1, 4))
    Qnew = np.zeros((1, 4))
    r2d = 57.2958
    
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
    while flg != 1:
        try:
            gyr_prev, acc_prev = input_queue.get(timeout = 0.01)
            gyr_prev = np.array([gyr_prev],dtype="float")
            acc_prev = np.array([acc_prev],dtype="float")
            Qold = acc2q(acc_prev) # Get the first quaternion state array by converting accelerometer data into a quaternion
            #ekf = EKF(frequency=17.3,frame='ENU',q0=Qold,noises=[0.1**2, 0.15**4, 0.8**2]) # EKF init from old code
            ekf = EKF(frequency=95,frame='ENU',q0=Qold,noises=[0.1**2, 0.15**4, 0.8**2]) # Initialise EKF function. Noise = [Process covariance, Measurement covariance]
            init_angles = q2rpy(Qold)*r2d
            smooth_roll_prev = init_angles[0]
            smooth_pitch_prev = init_angles[1]
            smooth_yaw_prev = init_angles[2]
            flg = 1
        except queue.Empty:
            continue
    ##### Initialise EKF end #####

    while not stop_event.is_set():
        try:
            gyro, acc = input_queue.get(timeout = 0.01)
            gyro = np.array([gyro],dtype="float")
            acc = np.array([acc],dtype="float")
            
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
            roll = angles[0]
            pitch = angles[1]
            yaw = angles[2]

            ##### Simple smoothening filter #####
            smooth_roll_cur =  salpha * smooth_roll_prev + (1 - salpha) * roll
            smooth_roll_prev = smooth_roll_cur
            smooth_pitch_cur =  salpha * smooth_pitch_prev + (1 - salpha) * pitch
            smooth_pitch_prev = smooth_pitch_cur
            smooth_yaw_cur =  salpha * smooth_yaw_prev + (1 - salpha) * yaw
            smooth_yaw_prev = smooth_yaw_cur

            angles = [smooth_roll_cur, smooth_pitch_cur, smooth_yaw_cur]
            ##########
            
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

            output_queue.put((angles[0],angles[1],angles[2]))
            input_queue.task_done()
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
                #ax /= 16384
                #ay /= 16384
                #az /= 16384

                # Gyroscope scaling (assuming scale of 262.4 LSB/deg/s from datasheet)
                #gyroscale = 1 / 262.4
                gyroscale = 1
                pitch_g = gx  #gyroscale * math.pi / 180  # Convert to radians
                roll_g = gy # gyroscale * math.pi / 180

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

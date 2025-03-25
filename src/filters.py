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
import csv
#import matplotlib.pyplot as plt
#import matplotlib.animation as animation
#from matplotlib import style
import REKF as rek
from scipy import signal
##### EKF libraries end #####
from scipy.signal import butter, filtfilt


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

def audio_bandpass_high_F_filter(input_queue, output_queue, stop_event):
    # Bandpass filter parameters (same as in your example)
    SAMPLE_RATE = 16500  # Sampling rate (Hz)
    LOWCUT = 7000.0      # Low-frequency cutoff (Hz)
    HIGHCUT = 8000.0     # High-frequency cutoff (Hz)
    FILTER_ORDER = 5
    
    # Pre-compute filter coefficients
    nyquist = 0.5 * SAMPLE_RATE
    low = LOWCUT / nyquist
    high = HIGHCUT / nyquist
    b, a = butter(FILTER_ORDER, [low, high], btype='band')
    
    while not stop_event.is_set():
        try:
            # Get audio data from input queue
            task = input_queue.get(timeout=0.01)
            
            # Add code here - Apply bandpass filter
            if isinstance(task, np.ndarray):
                # Apply zero-phase Butterworth bandpass filter
                filtered_audio = filtfilt(b, a, task)
                
                # Put filtered data in output queue
                output_queue.put(filtered_audio)
            else:
                # If it's not a numpy array, pass through unchanged
                output_queue.put(task)
                
            input_queue.task_done()
        except queue.Empty:
            continue
    
    print("audio_bandpass_high_F_filter_stopped")

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

            fields=[angles[0],angles[1],angles[2]]
            with open(r'foo.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow(fields)

            output_queue.put((angles[0],angles[1],angles[2]))
            input_queue.task_done()
        except queue.Empty:
            continue
    print("EKF_stopped")

def REK_filter(input_queue, output_queue, stop_event):
    print("rek_filter run")
    ##### Setup your variables and flags str #####
    gyr_alpha = 0.05
    acc_alpha = 0.005
    salpha = 0.00 # Originally 0.65, 0.75 for demo
    flg = 0
    
    gyr_prev = []
    acc_prev = []
    gyr_cur = []
    acc_cur = []

    Qold = np.zeros((1, 4))
    Qnew = np.zeros((1, 4))
    r2d = 57.2958

    maxavg = 50 # Max array size. Essential tremors 4-8Hz
    movingavg = np.array([[5]*3]*maxavg) # Array for moving average 
    pointer = 0

    ##### Setup your variables and flags end #####

    ##### Initialise REKF str #####
    while flg != 1:
        try:
            gyr_prev, acc_prev = input_queue.get(timeout = 0.01)
            gyr_prev = np.array([gyr_prev],dtype="float")
            acc_prev = np.array([acc_prev],dtype="float")
            Qold = acc2q(acc_prev) # Get the first quaternion state array by converting accelerometer data into a quaternion
            #ekf = EKF(frequency=17.3,frame='ENU',q0=Qold,noises=[0.1**2, 0.15**4, 0.8**2]) # EKF init from old code 0.6**2
            rekf = rek.OREKF(frequency=95,frame='ENU',q0=Qold,noises=[0.1**2, 0.15**4, 0.8**2]) # Initialise EKF function. Noise = [Process covariance, Measurement covariance]
            init_angles = q2rpy(Qold)*r2d
            smooth_roll_prev = init_angles[0]
            smooth_pitch_prev = init_angles[1]
            smooth_yaw_prev = init_angles[2]
            flg = 1
        except queue.Empty:
            continue
    ##### Initialise REKF end #####

    while not stop_event.is_set():
        try:
            gyro, acc = input_queue.get(timeout = 0.01)
            gyro = np.array([gyro],dtype="float")
            acc = np.array([acc],dtype="float")
            accavg = math.sqrt( (acc[0,0]*acc[0,0]) + (acc[0,1]*acc[0,1]) + (acc[0,2]*acc[0,2]) )
            #print(accavg)
            import control as ct
            #print(f"acc: {acc[0]:8.4f}, gyr: {gyro[0]:8.4f}")

            # accsat = ct.saturation_nonlinearity(5)
            # acc = accsat(acc)
            #print(gyro)
            
            #Your code

            ##### Pre-filter str #####
            satflag = 0
            #if( (abs(acc[0,0] - acc_prev[0,0]) > 1.5) or (abs(acc[0,1] - acc_prev[0,1]) > 1.5) or (abs(acc[0,2] - acc_prev[0,2]) > 1.5) ):
            # if( (abs(acc[0,0]) > 5) or (abs(acc[0,1]) > 5) or (abs(acc[0,2]) > 5) ):
            #     satflag = 1
            #     salpha = 0.98
            #     print("sat")
            # else:
            #     salpha = 0.05

            acc_cur = (acc_alpha * acc_prev) + ((1 - acc_alpha) * (acc))
            gyr_cur = (gyr_alpha * gyr_prev) + ((1 - gyr_alpha) * (gyro))

            # accsat  = ct.saturation_nonlinearity(acc_prev.all()*1.5,acc_prev.all()*0.5)
            # gyrsat  = ct.saturation_nonlinearity(gyr_prev.all()*1.5,gyr_prev.all()*0.5)

            # acc_cur = (acc_alpha * acc_prev) + ((1 - acc_alpha) * accsat(acc))
            # gyr_cur = (gyr_alpha * gyr_prev) + ((1 - gyr_alpha) * gyrsat(gyro))

            # acc_cur = accsat(acc)
            # gyr_cur = gyrsat(gyro)

            acc_prev = acc_cur
            gyr_prev = gyr_cur
            ##### Pre-filter end #####

            ##### REKF update str #####
            # if( satflag == 1 ):
            #     Qnew = Qold
            #     print("sat")
            # else:
            #     Qnew = rekf.rupdate(Qold, gyr_cur[0], acc_cur[0])
            Qnew = rekf.rupdate(Qold, gyr_cur[0], acc_cur[0]) # Update EKF function. See: https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/ekf.py#L1336
            Qold = Qnew

            angles = q2rpy(Qnew)*r2d # Get euler angles from quaternions 
            roll = angles[0]
            pitch = angles[1]
            yaw = angles[2]

            ##### Simple smoothening filter #####
            smooth_roll_cur =  salpha * smooth_roll_prev + (1 - salpha) * roll
            # smooth_roll_prev = smooth_roll_cur
            smooth_pitch_cur =  salpha * smooth_pitch_prev + (1 - salpha) * pitch
            # smooth_pitch_prev = smooth_pitch_cur
            smooth_yaw_cur =  salpha * smooth_yaw_prev + (1 - salpha) * yaw
            # smooth_yaw_prev = smooth_yaw_cur

            # rollsat  = ct.saturation_nonlinearity(smooth_roll_prev+0.8,smooth_roll_prev-0.8)
            # pitchsat  = ct.saturation_nonlinearity(smooth_pitch_prev+0.8,smooth_pitch_prev-0.8)
            # yawsat  = ct.saturation_nonlinearity(smooth_yaw_prev+0.8,smooth_yaw_prev-0.8)

            # smooth_roll_cur = rollsat(roll)
            # smooth_roll_prev = smooth_roll_cur
            # smooth_pitch_cur = pitchsat(pitch)
            # smooth_pitch_prev = smooth_pitch_cur
            # smooth_yaw_cur = yawsat(yaw)
            # smooth_yaw_prev = smooth_yaw_cur

            angles = [smooth_roll_cur, smooth_pitch_cur, smooth_yaw_cur]
            #print(angles)
            movingavg[pointer] = angles
            movingmean = movingavg.mean(axis=0,dtype=np.float64)
            #print(movingavg.mean(axis=0,dtype=np.float64))
            # print("var:")
            movingvar = movingavg.var(axis=0,dtype=np.float64)
            # print(movingavg.var(axis=0,dtype=np.float64))
            pointer = pointer+1
            if(pointer>=maxavg):
                pointer = 0

            #if( ( abs(angles[0]-movingmean[0]) > (3*movingvar[0]) ) or ( abs(angles[1]-movingmean[1]) > (3*movingvar[1]) ) or ( abs(angles[2]-movingmean[2]) > (3*movingvar[2]) ) ):
            # if( ( abs(angles[0]-movingmean[0]) > (3) ) or ( abs(angles[1]-movingmean[1]) > (3) ) or ( abs(angles[2]-movingmean[2]) > (3) ) ):
            if( (movingvar[0] > 25) or (movingvar[1] > 25) or (movingvar[2] > 25) ):
                #print(movingvar)
                angles = movingmean
            # # if( abs(angles[2]-smooth_yaw_prev) > abs(angles[2]-movingmean[2]) ):
            #     print("bruh")
            #     #angles = [i * 0.9 for i in movingmean] + [j * 0.1 for j in angles]
            #     angles = movingmean
            # #angles = [i * 0.5 for i in movingmean] + [j * 0.5 for j in angles]

            smooth_roll_prev = smooth_roll_cur
            smooth_pitch_prev = smooth_pitch_cur
            smooth_yaw_prev = smooth_yaw_cur
            ##########
            
            # print(f"Q: {[f'{(100*x):8.2f}' for x in Qold]}, E: {[f'{(100*x):8.2f}' for x in angles]}") # Print quat and euler output of update
            ##### REKF update end #####            

            # fields=[abs(angles[0]),abs(angles[1]),abs(angles[2])]
            # with open(r'foo.csv', 'a') as f:
            #     writer = csv.writer(f)
            #     writer.writerow(fields)

            output_queue.put((angles[0],angles[1],angles[2]))
            input_queue.task_done()
        except queue.Empty:
            continue
    print("REKF_stopped")
def TREK_filter(input_queue, output_queue, stop_event):
    print("trek_filter run")
    ##### Setup your variables and flags str #####
    gyr_alpha = 0.05
    acc_alpha = 0.005
    salpha = 0.00 # Originally 0.65, 0.75 for demo
    flg = 0
    
    gyr_prev = []
    acc_prev = []
    gyr_cur = []
    acc_cur = []

    Qold = np.zeros((1, 4))
    Qnew = np.zeros((1, 4))
    r2d = 57.2958

    maxavg = 50 # Max array size. Essential tremors 4-8Hz
    movingavg = np.array([[5]*3]*maxavg) # Array for moving average 
    pointer = 0

    ##### Setup your variables and flags end #####

    ##### Init low-pass filter #####
    # b, a = signal.butter(2, 1.5, fs=100) # 3rd order, cut-off = 3.5Hz
    #b, a = signal.butter(3, 2.5, fs=100) # 3rd order, cut-off = 2.5Hz < 15ms 4x attenuation
    # b, a = signal.butter(2, 1.5, fs=100) # 3rd order, cut-off = 1.5Hz < 10ms 10x attenuation
    #b, a = signal.butter(6, 1.8, fs=100) # 3rd order, cut-off = 3.5Hz
    # b, a = signal.ellip(3, 1, 30, 2, fs=100) # 3rd order, cut-off
    # b, a = signal.ellip(3, 0.1, 100, 1.5, fs=100) # 3rd order, cut-off

    # b, a = signal.ellip(3, 0.1, 50, [2.3,12], btype='bandstop', fs=100) # bandstop
    b, a = signal.ellip(3, 0.1, 60, [1.2,14], btype='bandstop', fs=100) # bandstop

    from scipy.signal import bode
    #w, mag, phase = bode(b,a)
    #print(phase)
    ##### #####

    ##### Initialise REKF str #####
    while flg != 1:
        try:
            gyr_prev, acc_prev = input_queue.get(timeout = 0.01)
            gyr_prev = np.array([gyr_prev],dtype="float")
            acc_prev = np.array([acc_prev],dtype="float")
            Qold = acc2q(acc_prev) # Get the first quaternion state array by converting accelerometer data into a quaternion
            #ekf = EKF(frequency=17.3,frame='ENU',q0=Qold,noises=[0.1**2, 0.15**4, 0.8**2]) # EKF init from old code 0.6**2
            rekf = rek.OREKF(frequency=95,frame='ENU',q0=Qold,noises=[0.1**2, 0.15**4, 0.8**2]) # Initialise EKF function. Noise = [Process covariance, Measurement covariance]
            init_angles = q2rpy(Qold)*r2d
            smooth_roll_prev = init_angles[0]
            smooth_pitch_prev = init_angles[1]
            smooth_yaw_prev = init_angles[2]
            zr = signal.lfilter_zi(b, a) * smooth_roll_prev
            zp = signal.lfilter_zi(b, a) * smooth_pitch_prev
            zy  = signal.lfilter_zi(b, a) * smooth_yaw_prev
            flg = 1
        except queue.Empty:
            continue
    ##### Initialise REKF end #####

    while not stop_event.is_set():
        try:
            gyro, acc = input_queue.get(timeout = 0.01)
            gyro = np.array([gyro],dtype="float")
            acc = np.array([acc],dtype="float")
            accavg = math.sqrt( (acc[0,0]*acc[0,0]) + (acc[0,1]*acc[0,1]) + (acc[0,2]*acc[0,2]) )
            #print(accavg)
            import control as ct
            #print(f"acc: {acc[0]:8.4f}, gyr: {gyro[0]:8.4f}")

            # accsat = ct.saturation_nonlinearity(5)
            # acc = accsat(acc)
            #print(gyro)
            
            #Your code

            ##### Pre-filter str #####
            satflag = 0
            #if( (abs(acc[0,0] - acc_prev[0,0]) > 1.5) or (abs(acc[0,1] - acc_prev[0,1]) > 1.5) or (abs(acc[0,2] - acc_prev[0,2]) > 1.5) ):
            # if( (abs(acc[0,0]) > 5) or (abs(acc[0,1]) > 5) or (abs(acc[0,2]) > 5) ):
            #     satflag = 1
            #     salpha = 0.98
            #     print("sat")
            # else:
            #     salpha = 0.05

            acc_cur = (acc_alpha * acc_prev) + ((1 - acc_alpha) * (acc))
            gyr_cur = (gyr_alpha * gyr_prev) + ((1 - gyr_alpha) * (gyro))

            # accsat  = ct.saturation_nonlinearity(acc_prev.all()*1.5,acc_prev.all()*0.5)
            # gyrsat  = ct.saturation_nonlinearity(gyr_prev.all()*1.5,gyr_prev.all()*0.5)

            # acc_cur = (acc_alpha * acc_prev) + ((1 - acc_alpha) * accsat(acc))
            # gyr_cur = (gyr_alpha * gyr_prev) + ((1 - gyr_alpha) * gyrsat(gyro))

            # acc_cur = accsat(acc)
            # gyr_cur = gyrsat(gyro)

            acc_prev = acc_cur
            gyr_prev = gyr_cur

            ##### Pre-filter end #####

            ##### Gyro biasing str #####
            
            # Theory: Push incoming gyro data into an array. Find the gradient of change. If change < threshold, set gyro = 0. If change > 0, gyro unaffected.

            movingavg[pointer] = gyro
            gyrochange = movingavg.max(axis=0,dtype=np.float64) - movingavg.min(axis=0,dtype=np.float64)
            

            if( sum(n > 1 for n in gyrochange) > 0):
                gyro = np.full((3, 1), 0)

            pointer = pointer+1
            if(pointer>=maxavg):
                pointer = 0
            #####

            ##### REKF update str #####
            # if( satflag == 1 ):
            #     Qnew = Qold
            #     print("sat")
            # else:
            #     Qnew = rekf.rupdate(Qold, gyr_cur[0], acc_cur[0])
            Qnew = rekf.rupdate(Qold, gyr_cur[0], acc_cur[0]) # Update EKF function. See: https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/ekf.py#L1336
            Qold = Qnew

            angles = q2rpy(Qnew)*r2d # Get euler angles from quaternions 
            roll = angles[0]
            pitch = angles[1]
            yaw = angles[2]

            ##### Simple smoothening filter #####
            smooth_roll_cur =  salpha * smooth_roll_prev + (1 - salpha) * roll
            # smooth_roll_prev = smooth_roll_cur
            smooth_pitch_cur =  salpha * smooth_pitch_prev + (1 - salpha) * pitch
            # smooth_pitch_prev = smooth_pitch_cur
            smooth_yaw_cur =  salpha * smooth_yaw_prev + (1 - salpha) * yaw
            # smooth_yaw_prev = smooth_yaw_cur

            # rollsat  = ct.saturation_nonlinearity(smooth_roll_prev+0.8,smooth_roll_prev-0.8)
            # pitchsat  = ct.saturation_nonlinearity(smooth_pitch_prev+0.8,smooth_pitch_prev-0.8)
            # yawsat  = ct.saturation_nonlinearity(smooth_yaw_prev+0.8,smooth_yaw_prev-0.8)

            # smooth_roll_cur = rollsat(roll)
            # smooth_roll_prev = smooth_roll_cur
            # smooth_pitch_cur = pitchsat(pitch)
            # smooth_pitch_prev = smooth_pitch_cur
            # smooth_yaw_cur = yawsat(yaw)
            # smooth_yaw_prev = smooth_yaw_cur

            ##### Str low-pass filtering #####
            # Set intial condiition 
            # zr = signal.lfilter_zi(b, a) * roll
            # zp = signal.lfilter_zi(b, a) * pitch
            # zy  = signal.lfilter_zi(b, a) * yaw

            filangles = np.empty(3)

            #print(angles)
            filangles[0], zr  = signal.lfilter(b, a, [roll], zi=zr)
            filangles[1], zp = signal.lfilter(b, a, [pitch], zi=zp)
            filangles[2], zy   = signal.lfilter(b, a, [yaw], zi=zy)
            #print(filroll.tolist())
            #print(type(filroll) is np.ndarray)
            ##### End low-pass filtering #####

            # angles = [smooth_roll_cur, smooth_pitch_cur, smooth_yaw_cur]
            # #print(angles)
            # movingavg[pointer] = angles
            # movingmean = movingavg.mean(axis=0,dtype=np.float64)
            # #print(movingavg.mean(axis=0,dtype=np.float64))
            # # print("var:")
            # movingvar = movingavg.var(axis=0,dtype=np.float64)
            # # print(movingavg.var(axis=0,dtype=np.float64))
            # pointer = pointer+1
            # if(pointer>=maxavg):
            #     pointer = 0

            # #if( ( abs(angles[0]-movingmean[0]) > (3*movingvar[0]) ) or ( abs(angles[1]-movingmean[1]) > (3*movingvar[1]) ) or ( abs(angles[2]-movingmean[2]) > (3*movingvar[2]) ) ):
            # # if( ( abs(angles[0]-movingmean[0]) > (3) ) or ( abs(angles[1]-movingmean[1]) > (3) ) or ( abs(angles[2]-movingmean[2]) > (3) ) ):
            # if( (movingvar[0] > 25) or (movingvar[1] > 25) or (movingvar[2] > 25) ):
            #     #print(movingvar)
            #     angles = movingmean
            # # # if( abs(angles[2]-smooth_yaw_prev) > abs(angles[2]-movingmean[2]) ):
            # #     print("bruh")
            # #     #angles = [i * 0.9 for i in movingmean] + [j * 0.1 for j in angles]
            # #     angles = movingmean
            # # #angles = [i * 0.5 for i in movingmean] + [j * 0.5 for j in angles]

            smooth_roll_prev = smooth_roll_cur
            smooth_pitch_prev = smooth_pitch_cur
            smooth_yaw_prev = smooth_yaw_cur
            ##########
            
            # print(f"Q: {[f'{(100*x):8.2f}' for x in Qold]}, E: {[f'{(100*x):8.2f}' for x in angles]}") # Print quat and euler output of update
            ##### REKF update end #####            

            fields=[angles[0],angles[1],angles[2],filangles[0],filangles[1],filangles[2]]
            with open(r'TREKfoo.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow(fields)

            output_queue.put((filangles[0],filangles[1],filangles[2]))
            #output_queue.put((filroll, filpitch, filyaw))
            input_queue.task_done()
        except queue.Empty:
            continue
    print("TREKF_stopped")

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

                # fields=[pitch_est,roll_est]
                # with open(r'compFilter.csv', 'a') as f:
                #     writer = csv.writer(f)
                #     writer.writerow(fields)

                # Output filtered angles to the output queue
                output_queue.put((pitch_est, roll_est))

                # Mark task as done
                input_queue.task_done()
        except queue.Empty:
            continue

    print("complementary_filter_stopped yay!!!")

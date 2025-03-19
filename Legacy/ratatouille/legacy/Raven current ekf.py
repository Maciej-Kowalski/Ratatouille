# from networks import omega
import serial
import numpy as np
import time
from ahrs.filters import EKF
import ahrs
from ahrs.common.orientation import acc2q
from ahrs.common.orientation import q2rpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

# Port of STM32.
Arduino =serial.Serial('COM5',115200)

predict_period = 0.01 # Unused (for now)
update_period = 0.05 # Time in seconds of update period for the EKF algorithm
predict_clock = 0.0 # Unused (for now)
update_clock = 0.0 # Variable used to keep track of update_period
gyr_alpha = 0.1 # Alpha value of IIR filter for gyroscope 
acc_alpha = 0.5 # Alpha value of IIR filter for accelerometer 

# ekf = EKF()

# num_samples = 1000              # Assuming sensors have 1000 samples each

# Q = np.zeros((num_samples, 4))  # Allocate array for quaternions

# Q[0] = acc2q(acc_data[0])       # First sample of tri-axial accelerometer

# for t in range(1, num_samples):

#     Q[t] = ekf.update(Q[t-1], gyr_data[t], acc_data[t])

# Initialise
gyr_data = [] # Stores the XYZ gyroscope variables just gotten from STM32 
acc_data = [] # Stores the XYZ accelerometer variables just gotten from STM32 
gyr_prev = [] # Stores the previous XYZ gyroscope variables 
acc_prev = [] # Stores the previous XYZ accelerometer variables 
gyr_cur = [] # Stores the current XYZ gyroscope variables 
acc_cur = [] # Stores the current XYZ accelerometer variables 
msg = 0 # Flag which ensures initial IMU data is receieved
plot_count = 0
wplt = [] # Stores w quaternion for plotting
xplt = [] # and so on...
yplt = []
zplt = []

salpha = 0.65 # Simple smoothening filter alpha value
smooth_pitch_prev = 0 # Previous value of smoothening filter
smooth_pitch_cur = 0 # Current value of smoothening filter
syplt = [] # Stores smoothening filter value for plotting
nofily = 0  # Stores y accelerometer data from STM32 as a quaternion
nofilyplt = []  # Stores y accelerometer data for plotting

##### Plot handling #####
plt.axis([0, 9.8, -1, 1])
a = np.arange(0, 10, 0.05)
plt.grid()
plt.title("Euler Output of Extended Kalman Filter")
plt.xlabel("Time (s)")
plt.ylabel("Quaternion Value")
###########

# Initialises an empty np array which stores previous and updated EKF quaternion values
Qold = np.zeros((1, 4))
Qnew = np.zeros((1, 4))

# Get first IMU data
while(msg == 0):
    # Get IMU data. Expects in the format:
    # AX, AY, AZ, GX, GY, GZ 
    data  =  str(Arduino.readline().decode('ascii').rstrip()) # Read the data
    holder = data.split(",") # Split the values by the ","
    floatd = [float(string) for string in holder] # Convert strings into float values
    acc_data = floatd[:len(floatd)//2] # Get the first half of the data array
    acc_prev = np.array([acc_data],dtype=float) # Store the accelerometer datainto an np array: [[AX, AY, AZ]]
    #print(acc_arr)
    gyr_data = floatd[len(floatd)//2:] # Get the second half of the data array
    gyr_prev = np.array([gyr_data],dtype=float) # Store the gyroscope datainto an np array: [[GX, GY, GZ]]
    #print(gyr_arr)
    acc_prev = acc_prev*9.81 # Convert from g to m/s
    gyr_prev = gyr_prev*0.0174533 # Convert from degree/s to rad/s
    #print(acc_prev)
    ekf = EKF(frequency=18,frame='ENU') # Initialise EKF function
    Qold = acc2q(acc_prev[0]) # Get the first quaternion state array by converting accelerometer data into a quaternion
    print(Qold)
    smooth_pitch_prev = Qold[1]
    msg = 1 # Raise flag, exiting initial loop

# While
while 1:
    ##### Read IMU data from STM32 and store accelerometer and gyroscope data into seperate arrays ##### 
    # Get IMU data
    data  =  str(Arduino.readline().decode('ascii').rstrip())
    print(data)
    holder = data.split(",")
    floatd = [float(string) for string in holder]
    acc_data = floatd[:len(floatd)//2]
    acc_arr = np.array([acc_data],dtype=float)
    print(acc_arr)
    gyr_data = floatd[len(floatd)//2:]
    gyr_arr = np.array([gyr_data],dtype=float)
    #print(gyr_arr)
    acc_arr = acc_arr*9.81
    gyr_arr = gyr_arr*0.0174533
    #print(acc_arr)
    nofily = acc2q(acc_arr[0])[1]
    ##########

    ##### IIR Pre-filter #####
    acc_cur = (acc_alpha * acc_prev) + ((1 - acc_alpha) * (acc_arr))
    gyr_cur = (gyr_alpha * gyr_prev) + ((1 - gyr_alpha) * (gyr_arr))

    acc_prev = acc_cur
    gyr_prev = gyr_cur
    #####

    ##### EKF loop #####
    if time.perf_counter() - update_clock >= update_period: # If amount of time passed is above the update_period threshold
        #ekf = EKF(gyr_cur, acc_cur,frequency=20.0,frame='ENU')
        #ekf = EKF(gyr_cur, acc_cur,frequency=20.0,frame='ENU')
        #print(ekf.Q)
        Qnew = ekf.update(Qold, gyr_cur[0], acc_cur[0]) # Update EKF function
        #print(Qnew)
        Qold = Qnew
        print(Qnew)

        ##### Plotting #####
        w = Qold[0] 
        x = Qold[1] # Roll
        y = Qold[2] # Pitch

        ##### Simple smoothening filter #####
        smooth_pitch_cur =  salpha * smooth_pitch_prev + (1 - salpha) * y
        smooth_pitch_prev = smooth_pitch_cur
        ##########

        z = Qold[3] 
        if(plot_count < 200):
            wplt.append(w)
            xplt.append(x)
            yplt.append(y)
            zplt.append(z)
            syplt.append(smooth_pitch_cur)
            nofilyplt.append(nofily)
            plot_count = 1+ plot_count
        elif(plot_count == 200):
            plt.plot(a, wplt, '-', label = "w")
            plt.plot(a, xplt, '-', label = "x")
            plt.plot(a, yplt, '-', label = "y")
            plt.plot(a, zplt, '-', label = "z")
            plt.plot(a, syplt, '-', label = "Smoothened y")
            plt.plot(a, nofilyplt, '-', label = "Unfiltered y")
            plt.xticks(np.arange(0, 10, 0.5))
            plt.yticks(np.arange(-1, 1, 0.2))
            plt.legend()
            plt.show()
            plot_count = 1 + plot_count
        ##########

        update_clock = time.perf_counter()
    #####
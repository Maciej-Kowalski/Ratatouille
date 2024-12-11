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

Arduino =serial.Serial('COM7',115200)

predict_period = 0.01
update_period = 0.01
predict_clock = 0.0
update_clock = 0.0
gyr_alpha = 0.1
acc_alpha = 0.5

# ekf = EKF()

# num_samples = 1000              # Assuming sensors have 1000 samples each

# Q = np.zeros((num_samples, 4))  # Allocate array for quaternions

# Q[0] = acc2q(acc_data[0])       # First sample of tri-axial accelerometer

# for t in range(1, num_samples):

#     Q[t] = ekf.update(Q[t-1], gyr_data[t], acc_data[t])

# Initialise
gyr_data = []
acc_data = []
gyr_prev = []
acc_prev = []
gyr_cur = []
acc_cur = []
msg = 0
plot_count = 0
wplt = []
xplt = []
yplt = []
zplt = []
plt.axis([0, 4.9, -1, 1])
a = np.arange(0, 5, 0.1)
plt.grid()
plt.title("Quaternion Output of Extended Kalman Filter")
plt.xlabel("Time (s)")
plt.ylabel("Rotation (°)")

# Get first IMU data
while(msg == 0):
    # Get IMU data
    data  =  str(Arduino.readline().decode('ascii').rstrip())   #read the data
    holder = data.split(",")
    floatd = [float(string) for string in holder]
    acc_data = floatd[len(floatd)//2:]
    acc_prev = np.array([acc_data],dtype=float)
    #print(acc_arr)
    gyr_data = floatd[:len(floatd)//2]
    gyr_prev = np.array([gyr_data],dtype=float)
    #print(gyr_arr)
    acc_prev = acc_prev*9.81
    gyr_prev = gyr_prev*0.0174533
    #print(acc_arr)
    msg = 1

# While
while 1:
    # Get IMU data
    data  =  str(Arduino.readline().decode('ascii').rstrip())   #read the data
    holder = data.split(",")
    floatd = [float(string) for string in holder]
    acc_data = floatd[len(floatd)//2:]
    acc_arr = np.array([acc_data],dtype=float)
    #print(acc_arr)
    gyr_data = floatd[:len(floatd)//2]
    gyr_arr = np.array([gyr_data],dtype=float)
    #print(gyr_arr)
    acc_arr = acc_arr*9.81
    gyr_arr = gyr_arr*0.0174533
    #print(acc_arr)

    # Pre-filter
    acc_cur = (acc_alpha * acc_prev) + ((1 - acc_alpha) * (acc_arr))
    gyr_cur = (gyr_alpha * gyr_prev) + ((1 - gyr_alpha) * (gyr_arr))

    acc_prev = acc_cur
    gyr_prev = gyr_cur

    if time.perf_counter() - update_clock >= update_period:
        ekf = EKF(gyr_cur, acc_cur,frequency=100.0,frame='ENU')
        #ekf = EKF(gyr_cur, acc_cur,frequency=100.0)
        print(ekf.Q)
        w = ekf.Q[0,0] 
        x = ekf.Q[0,1] 
        y = ekf.Q[0,2] 
        z = ekf.Q[0,3] 
        if(plot_count < 50):
            wplt.append(w)
            xplt.append(x)
            yplt.append(y)
            zplt.append(z)
            plot_count = 1+ plot_count
        elif(plot_count == 50):
            plt.plot(a, wplt, '-x', label = "w")
            plt.plot(a, xplt, '-x', label = "x")
            plt.plot(a, yplt, '-x', label = "y")
            plt.plot(a, zplt, '-x', label = "z")
            plt.xticks(np.arange(0, 5, 0.5))
            plt.yticks(np.arange(-1, 1, 0.2))
            plt.legend()
            plt.show()
            plot_count = 1 + plot_count
        update_clock = time.perf_counter()
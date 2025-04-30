from numpy.fft import fft, ifft
import csv
import matplotlib.pyplot as plt
import numpy as np

data = csv.reader(open('TREKffoo.csv', 'rt'), delimiter=',')
r, p, y, fr, fp, fy = [], [], [], [], [], [] 
for row in data:
    r.append(round(float(row[0]),3))
    p.append(round(float(row[1]),3))
    y.append(round(float(row[2]),3))
    #np.append(y, round(float(row[2]),3))
    fr.append(round(float(row[3]),3))
    fp.append(round(float(row[4]),3))
    fy.append(round(float(row[5]),3))
#####

sample_rate = 100

# This returns the fourier transform coeficients as complex numbers
transformed_y = np.fft.fft(y)

# Take the absolute value of the complex numbers for magnitude spectrum
freqs_magnitude = np.abs(transformed_y)

# Create frequency x-axis that will span up to sample_rate
freq_axis = np.linspace(0, sample_rate, len(freqs_magnitude))

transformed_fy = np.fft.fft(fy)
freqs_fmagnitude = np.abs(transformed_fy)

# Plot frequency domain
plt.plot(freq_axis, freqs_magnitude, label='Yaw')
plt.plot(freq_axis, freqs_fmagnitude, label='Filtered Yaw')
plt.grid()
plt.legend()
plt.title("Robust Extended Kalman with Elliptic Bandstop Filter of Variable 4-6Hz Tremor Emulation")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
#plt.xlim(0, 100)
plt.show()

#####

# sp = np.fft.fft(y)

# freq = np.fft.fftfreq(y.shape[-1])

# plt.plot(freq, sp.real, freq, sp.imag)
# plt.show()

#####

# sr = 100 # 100 hz
# # sampling interval
# ts = 1.0/sr
# t = np.arange(0,1,ts)

# X = fft(y)
# N = len(y)
# n = np.arange(N)
# T = N/sr
# freq = n/T 

# plt.figure(figsize = (12, 6))
# plt.subplot(121)

# plt.stem(freq, np.abs(X), 'b', \
#          markerfmt=" ", basefmt="-b")
# plt.xlabel('Freq (Hz)')
# plt.ylabel('FFT Amplitude |X(freq)|')
# plt.xlim(0, 10)

# plt.subplot(122)
# plt.plot(t, ifft(X), 'r')
# plt.xlabel('Time (s)')
# plt.ylabel('Amplitude')
# plt.tight_layout()
# plt.show()
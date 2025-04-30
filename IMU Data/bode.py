from scipy import signal
from scipy.signal import bode
import matplotlib.pyplot as plt

b, a = signal.butter(3, 3.5, fs=100) # 3rd order, cut-off = 3.5Hz
w, mag, phase = bode(b,a)
print((phase/360)*0.01)
# plt.figure()
# plt.semilogx(w,mag)
# plt.show()
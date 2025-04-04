import numpy as np
import matplotlib.pyplot as plt
import sounddevice as sd
from matplotlib.animation import FuncAnimation
import time
from pynput.mouse import Button, Controller

from matplotlib.ticker import AutoMinorLocator
from scipy.signal import butter, filtfilt

# ---------------- Parameters ----------------
SAMPLE_RATE = 4000  # Sampling rate (Hz)
DURATION = 4  # Seconds of data shown live
BUFFER_SIZE = int(SAMPLE_RATE * DURATION)

# Sliding-window click detection parameters
WINDOW_DURATION = 0.10  # Length of the window (seconds)
POSITIVE_THRESHOLD = 0.95  # Must go above this
NEGATIVE_THRESHOLD = -POSITIVE_THRESHOLD  # Must go below this
COUNT_THRESHOLD = 20  # Not used in this version? (We can do a simpler approach.)
RECTIFY_SIGNAL = False  # Do *not* rectify in the callback

# Bandpass filter parameters
LOWCUT = 600.0  # Low-frequency cutoff (Hz)
HIGHCUT = 1800.0  # High-frequency cutoff (Hz)
FILTER_ORDER = 5

# Derived
WINDOW_SIZE = int(WINDOW_DURATION * SAMPLE_RATE)

# Initialize audio buffer
audio_buffer = np.zeros(BUFFER_SIZE)
filtered_buffer = np.zeros(BUFFER_SIZE)

# Cooldown (to avoid registering many clicks in a row)
click_count = 0
last_click_time = 0.0
click_cooldown = 0.18  # seconds to wait before next click


# --------------- Click Detection ---------------
def detect_click_sliding_window(signal):
    """
    Instead of looking for an amplitude > 0.25 alone,
    we now check if the window has a max above +0.25 AND a min below -0.25.
    """
    if len(signal) < WINDOW_SIZE:
        window = signal
    else:
        window = signal[-WINDOW_SIZE:]

    window_max = np.max(window)
    window_min = np.min(window)

    # Condition: has it gone above +0.25 and below -0.25 in the same window?
    if window_max > POSITIVE_THRESHOLD and window_min < NEGATIVE_THRESHOLD:
        return True
    else:
        return False


# --------------- Bandpass Filter ---------------
def butter_bandpass_filter(data):
    """
    Apply a zero-phase Butterworth bandpass filter to 'data'.
    """
    nyquist = 0.5 * SAMPLE_RATE
    low = LOWCUT / nyquist
    high = HIGHCUT / nyquist
    b, a = butter(FILTER_ORDER, [low, high], btype='band')
    return filtfilt(b, a, data)  # zero-phase filtering


# --------------- Audio Callback ---------------
def audio_callback(indata, frames, time_info, status):
    global audio_buffer, filtered_buffer
    if status:
        print(f"Status: {status}")

    # Shift old samples left
    audio_buffer[:-frames] = audio_buffer[frames:]
    # Store new raw samples (no rectification, just raw)
    audio_buffer[-frames:] = indata[:, 0]

    # Apply filtering separately
    filtered_buffer[:] = butter_bandpass_filter(audio_buffer)


def left_click():
    mouse = Controller()
    mouse.click(Button.left, 1)


# --------------- Plot Update ---------------
def update_plot(frame):
    global click_count, last_click_time

    # 1) Update live time-domain plot with filtered data
    line_time.set_ydata(filtered_buffer)

    # 2) Click detection with +/- threshold
    now = time.time()
    if detect_click_sliding_window(filtered_buffer):
        # Only register a new click if cooldown has passed
        if now - last_click_time > click_cooldown:
            click_count += 1
            last_click_time = now
            print(f"Click {click_count} detected (window-based)!")
            left_click()


# --------------- Matplotlib Setup ---------------
fig, ax_time = plt.subplots(figsize=(10, 5))

x_time = np.linspace(0, DURATION, BUFFER_SIZE)
line_time, = ax_time.plot(x_time, filtered_buffer, label="Bandpass Filtered Signal")
ax_time.set_xlim(0, DURATION)
ax_time.set_ylim(-0.7, 0.7)  # Adjust as needed
ax_time.set_title("Live Time-Domain Signal (High Bandpass Filter)")
ax_time.set_xlabel("Time (s)")
ax_time.set_ylabel("Amplitude")
ax_time.legend()
ax_time.grid(True, linestyle="--", linewidth=0.5)

# More x-axis ticks
ax_time.set_xticks(np.arange(0, DURATION + 0.1, 0.2))
ax_time.xaxis.set_minor_locator(AutoMinorLocator(2))  # 2 minor ticks per major interval


# --------------- Main Execution ---------------
def main():
    stream = sd.InputStream(
        channels=2,
        samplerate=SAMPLE_RATE,
        callback=audio_callback
    )

    with stream:
        ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
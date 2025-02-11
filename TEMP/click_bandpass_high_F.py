import numpy as np
import matplotlib.pyplot as plt
import sounddevice as sd
from matplotlib.animation import FuncAnimation
import time
from pynput.mouse import Button, Controller
from collections import deque
from matplotlib.ticker import AutoMinorLocator
from scipy.signal import butter, filtfilt

# ---------------- Parameters ----------------
SAMPLE_RATE = 16500            # Sampling rate (Hz)
DURATION = 4                   # Seconds of data shown live (for plotting only)
PACKET_SIZE = 100              # Number of samples per incoming packet
WINDOW_PACKETS = 20            # Number of packets in our sliding window
WINDOW_SIZE = PACKET_SIZE * WINDOW_PACKETS  # Total samples in the sliding window

# Sliding-window click detection parameters
POSITIVE_THRESHOLD = 0.20      # Must exceed this value
NEGATIVE_THRESHOLD = -POSITIVE_THRESHOLD

# Bandpass filter parameters
LOWCUT = 7000.0                # Low-frequency cutoff (Hz)
HIGHCUT = 8000.0               # High-frequency cutoff (Hz)
FILTER_ORDER = 5

# Cooldown to avoid rapid repeated clicks
click_cooldown = 0.18          # seconds
click_count = 0
last_click_time = 0.0

# --------------- Global Buffer using deque ----------------
# Instead of one continuous audio_buffer, we now use a deque of packets.
packet_buffer = deque(maxlen=WINDOW_PACKETS)

# --------------- Bandpass Filter Function ----------------
def butter_bandpass_filter(data):
    nyquist = 0.5 * SAMPLE_RATE
    low = LOWCUT / nyquist
    high = HIGHCUT / nyquist
    b, a = butter(FILTER_ORDER, [low, high], btype='band')
    return filtfilt(b, a, data)

# --------------- Click Detection ----------------
def detect_click_sliding_window(signal):
    """
    Check if the window has a maximum above POSITIVE_THRESHOLD
    and a minimum below NEGATIVE_THRESHOLD.
    """
    window_max = np.max(signal)
    window_min = np.min(signal)
    return window_max > POSITIVE_THRESHOLD and window_min < NEGATIVE_THRESHOLD

# --------------- Click Action ----------------
def left_click():
    mouse = Controller()
    mouse.click(Button.left, 1)
    print("Click detected and performed!")

# --------------- Audio Callback ----------------
def audio_callback(indata, frames, time_info, status):
    if status:
        print(f"Status: {status}")
    # Treat the incoming data as one packet
    new_packet = indata[:, 0].copy()  # Ensure we copy the data
    packet_buffer.append(new_packet)
    # Note: filtering is deferred to update_plot()

# --------------- Plot Update ----------------
def update_plot(frame):
    global click_count, last_click_time

    # Only process if we have a full window (20 packets)
    if len(packet_buffer) < WINDOW_PACKETS:
        return

    # Concatenate packets (oldest first) into a single window
    window = np.concatenate(list(packet_buffer))
    
    # Apply bandpass filtering to the entire window
    filtered_window = butter_bandpass_filter(window)

    # Update live plot with the filtered data
    window_duration = WINDOW_PACKETS * PACKET_SIZE / SAMPLE_RATE
    x_window = np.linspace(0, window_duration, len(filtered_window))
    line_time.set_data(x_window, filtered_window)
    ax_time.set_xlim(0, window_duration)

    # Click detection on the filtered window
    now = time.time()
    if detect_click_sliding_window(filtered_window):
        if now - last_click_time > click_cooldown:
            click_count += 1
            last_click_time = now
            print(f"Click {click_count} detected (window-based)!")
            left_click()

# --------------- Matplotlib Setup ----------------
fig, ax_time = plt.subplots(figsize=(10, 5))
x_time = np.linspace(0, DURATION, WINDOW_SIZE)
# Initialize the plot with zeros (will be updated in update_plot)
line_time, = ax_time.plot(x_time, np.zeros(WINDOW_SIZE), label="Bandpass Filtered Signal")
ax_time.set_xlim(0, DURATION)
ax_time.set_ylim(-0.7, 0.7)
ax_time.set_title("Live Time-Domain Signal (High Bandpass Filter)")
ax_time.set_xlabel("Time (s)")
ax_time.set_ylabel("Amplitude")
ax_time.legend()
ax_time.grid(True, linestyle="--", linewidth=0.5)
ax_time.set_xticks(np.arange(0, DURATION + 0.1, 0.2))
ax_time.xaxis.set_minor_locator(AutoMinorLocator(2))

# --------------- Main Execution ----------------
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

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
from scipy.signal import butter, filtfilt, lfilter
from pynput.mouse import Button, Controller
import sounddevice as sd
import soundfile as sf
import threading


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
    # Parameters (same as in your example)
    SAMPLE_RATE = 16500  # Sampling rate (Hz)
    LOWCUT = 7000.0      # Low-frequency cutoff (Hz)
    HIGHCUT = 8000.0     # High-frequency cutoff (Hz)
    FILTER_ORDER = 5
    
    # Sliding-window click detection parameters
    WINDOW_DURATION = 0.05  # Length of the window (seconds)
    POSITIVE_THRESHOLD = 500  # Must go above this
    NEGATIVE_THRESHOLD = -POSITIVE_THRESHOLD  # Must go below this
    COUNT_THRESHOLD = 10  # Number of threshold crossings needed in window
    WINDOW_SIZE = int(WINDOW_DURATION * SAMPLE_RATE)
    
    # Cooldown variables
    click_count = 0
    last_click_time = 0.0
    click_cooldown = 0.40  # seconds to wait before next click
    
    # Pre-compute filter coefficients
    nyquist = 0.5 * SAMPLE_RATE
    low = LOWCUT / nyquist
    high = HIGHCUT / nyquist
    b, a = butter(FILTER_ORDER, [low, high], btype='band')
    
    # Buffer to maintain context for click detection
    buffer_duration = 0.5  # seconds of context to maintain
    buffer_size = int(buffer_duration * SAMPLE_RATE)
    context_buffer = np.zeros(buffer_size)
    
    # Click detection function with counting threshold
    def detect_click_sliding_window(signal):
        """
        Check if the window has enough threshold crossings and meets amplitude criteria.
        """
        if len(signal) < WINDOW_SIZE:
            window = signal
        else:
            window = signal[-WINDOW_SIZE:]

        window_max = np.max(window)
        window_min = np.min(window)

        # First check: amplitude criteria
        if window_max <= POSITIVE_THRESHOLD or window_min >= NEGATIVE_THRESHOLD:
            return False
            
        # Count threshold crossings
        # We'll count how many times the signal crosses from below to above positive threshold
        # and from above to below negative threshold
        crossing_count = 0
        
        for i in range(1, len(window)):
            # Positive threshold crossings
            if window[i-1] < POSITIVE_THRESHOLD and window[i] >= POSITIVE_THRESHOLD:
                crossing_count += 1
                
            # Negative threshold crossings
            if window[i-1] > NEGATIVE_THRESHOLD and window[i] <= NEGATIVE_THRESHOLD:
                crossing_count += 1
        
        # Return true if we have enough crossings
        return crossing_count >= COUNT_THRESHOLD
    
    # Mouse click function
    def left_click():
        mouse = Controller()
        mouse.click(Button.left, 1)
    
    while not stop_event.is_set():
        try:
            # Get audio data from input queue
            task = input_queue.get(timeout=0.01)
            
            if isinstance(task, np.ndarray):
                # Apply bandpass filter
                filtered_audio = filtfilt(b, a, task)
                
                # Update the context buffer with new filtered data
                if len(filtered_audio) < buffer_size:
                    # Shift buffer and add new data
                    context_buffer = np.roll(context_buffer, -len(filtered_audio))
                    context_buffer[-len(filtered_audio):] = filtered_audio
                else:
                    # If new data is larger than buffer, just use the newest portion
                    context_buffer = filtered_audio[-buffer_size:]
                
                # Check for click using the enhanced sliding window approach
                now = time.time()
                if detect_click_sliding_window(context_buffer):
                    # Only register a new click if cooldown has passed
                    if now - last_click_time > click_cooldown:
                        click_count += 1
                        last_click_time = now
                        print(f"Click {click_count} detected! ({time.strftime('%H:%M:%S')})")
                        left_click()
                
                # Put filtered data in output queue
                output_queue.put(filtered_audio)
            else:
                # If it's not a numpy array, pass through unchanged
                output_queue.put(task)
                
            input_queue.task_done()
        except queue.Empty:
            continue
        except Exception as e:
            print(f"Error in audio bandpass filter: {str(e)}")
    
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

def audio_bandpass_high_F_filter_mean(input_queue, output_queue, stop_event, sound_file="click_sound.wav"):
    import sounddevice as sd
    import soundfile as sf
    import threading
    
    # Parameters
    SAMPLE_RATE = 16500  # Sampling rate (Hz)
    LOWCUT = 7000.0      # Low-frequency cutoff (Hz)
    HIGHCUT = 8000.0     # High-frequency cutoff (Hz)
    FILTER_ORDER = 5
    
    # Simplified detection parameters
    WINDOW_DURATION = 0.05  # Length of the window (seconds)
    MEAN_THRESHOLD = 300    # Mean of absolute values must exceed this
    WINDOW_SIZE = int(WINDOW_DURATION * SAMPLE_RATE)
    
    # Cooldown variables
    click_count = 0
    last_click_time = 0.0
    click_cooldown = 0.40  # seconds to wait before next click
    
    # Pre-compute filter coefficients
    nyquist = 0.5 * SAMPLE_RATE
    low = LOWCUT / nyquist
    high = HIGHCUT / nyquist
    b, a = butter(FILTER_ORDER, [low, high], btype='band')
    
    # Smaller buffer for lower latency
    buffer_duration = 0.1  # Reduced from 0.5 seconds for lower latency
    buffer_size = int(buffer_duration * SAMPLE_RATE)
    context_buffer = np.zeros(buffer_size)
    
    # Load the sound file to play on click detection
    try:
        click_sound_data, click_sound_samplerate = sf.read(sound_file)
        print(f"Loaded sound file: {sound_file}")
    except Exception as e:
        print(f"Warning: Could not load sound file ({e}). Will use default click.")
        click_sound_data = None
    
    # Function to play sound in a separate thread
    def play_sound():
        try:
            if click_sound_data is not None:
                sd.play(click_sound_data, click_sound_samplerate)
            else:
                # Generate a simple beep if no sound file is available
                beep_duration = 0.1  # seconds
                beep_freq = 1000  # Hz
                t = np.linspace(0, beep_duration, int(beep_duration * 44100), False)
                beep = 0.5 * np.sin(2 * np.pi * beep_freq * t)
                sd.play(beep, 44100)
        except Exception as e:
            print(f"Error playing sound: {e}")
    
    # Simplified detection function using mean instead of threshold crossings
    def detect_click_mean_threshold(signal):
        """
        Check if the mean of absolute values in the window exceeds the threshold.
        """
        if len(signal) < WINDOW_SIZE:
            window = signal
        else:
            window = signal[-WINDOW_SIZE:]
        
        # Calculate mean of absolute values
        mean_abs = np.mean(np.abs(window))
        
        # Return True if mean exceeds threshold
        return mean_abs > MEAN_THRESHOLD
    
    # Mouse click function
    def left_click():
        mouse = Controller()
        mouse.click(Button.left, 1)
        
        # Play sound in a separate thread to avoid blocking
        sound_thread = threading.Thread(target=play_sound)
        sound_thread.daemon = True
        sound_thread.start()
    
    # Initialize filter state for lfilter
    zi = np.zeros(max(len(a), len(b)) - 1)
    
    print(f"Audio click detection active with mean threshold: {MEAN_THRESHOLD}")
    print("Listening for high-frequency sounds (7-8 kHz)...")
    
    while not stop_event.is_set():
        try:
            # Get audio data from input queue with minimal timeout
            try:
                task = input_queue.get(timeout=0.005)  # Reduced timeout
            except queue.Empty:
                continue
            
            if isinstance(task, np.ndarray):
                # Apply bandpass filter with persistent state for efficiency
                filtered_audio, zi = lfilter(b, a, task, zi=zi)
                
                # Update the context buffer with new filtered data
                if len(filtered_audio) < buffer_size:
                    # Shift buffer and add new data
                    context_buffer = np.roll(context_buffer, -len(filtered_audio))
                    context_buffer[-len(filtered_audio):] = filtered_audio
                else:
                    # If new data is larger than buffer, just use the newest portion
                    context_buffer = filtered_audio[-buffer_size:]
                
                # Check for click using the mean threshold approach
                now = time.time()
                mean_abs_value = np.mean(np.abs(context_buffer))
                
                if mean_abs_value > MEAN_THRESHOLD:
                    # Only register a new click if cooldown has passed
                    if now - last_click_time > click_cooldown:
                        click_count += 1
                        last_click_time = now
                        print(f"Click {click_count} detected! Mean: {mean_abs_value:.1f}")
                        left_click()  # This will also play the sound
                
                # Put filtered data in output queue
                output_queue.put(filtered_audio)
            else:
                # If it's not a numpy array, pass through unchanged
                output_queue.put(task)
                
            input_queue.task_done()
        except Exception as e:
            print(f"Error in audio bandpass filter: {str(e)}")
    
    print("audio_bandpass_high_F_filter_stopped")


def audio_bandpass_high_F_filter_durations(input_queue, output_queue, stop_event, sound_file=None):
    import sounddevice as sd
    import soundfile as sf
    import threading
    
    # Parameters
    SAMPLE_RATE = 16500  # Sampling rate (Hz)
    LOWCUT = 7000.0      # Low-frequency cutoff (Hz)
    HIGHCUT = 8000.0     # High-frequency cutoff (Hz)
    FILTER_ORDER = 5
    
    # Detection parameters
    WINDOW_DURATION = 0.15  # Length of the window (seconds)
    MEAN_THRESHOLD = 200    # Mean of absolute values must exceed this
    WINDOW_SIZE = int(WINDOW_DURATION * SAMPLE_RATE)
    
    # Cooldown variables
    click_count = 0
    last_click_time = 0.0
    click_cooldown = 0.40  # seconds to wait before next action sequence
    
    # For tracking consecutive windows above threshold
    consecutive_windows = 0
    right_button_held = False
    
    # Pre-compute filter coefficients
    nyquist = 0.5 * SAMPLE_RATE
    low = LOWCUT / nyquist
    high = HIGHCUT / nyquist
    b, a = butter(FILTER_ORDER, [low, high], btype='band')
    
    # Buffer to maintain context for detection
    buffer_duration = 0.1  # seconds
    buffer_size = int(buffer_duration * SAMPLE_RATE)
    context_buffer = np.zeros(buffer_size)
    
    # Load sound file if provided
    try:
        if sound_file:
            click_sound_data, click_sound_samplerate = sf.read(sound_file)
            print(f"Loaded sound file: {sound_file}")
        else:
            click_sound_data = None
    except Exception as e:
        print(f"Warning: Could not load sound file ({e}). Will use default sounds.")
        click_sound_data = None
    
    # Functions to play different sounds
    def play_sound(frequency=1000, duration=0.1, volume=0.5):
        try:
            if click_sound_data is not None:
                sd.play(click_sound_data, click_sound_samplerate)
            else:
                # Generate a simple beep
                t = np.linspace(0, duration, int(duration * 44100), False)
                beep = volume * np.sin(2 * np.pi * frequency * t)
                sd.play(beep, 44100)
        except Exception as e:
            print(f"Error playing sound: {e}")
    
    # Function to check if mean is above threshold
    def is_above_threshold(signal):
        if len(signal) < WINDOW_SIZE:
            window = signal
        else:
            window = signal[-WINDOW_SIZE:]
        
        mean_abs = np.mean(np.abs(window))
        return mean_abs > MEAN_THRESHOLD, mean_abs
    
    # Mouse control functions
    mouse = Controller()
    
    def left_click():
        mouse.click(Button.left)
        threading.Thread(target=play_sound, args=(1000, 0.1, 0.5), daemon=True).start()
        print("Left click")
    
    def left_double_click():
        mouse.click(Button.left, 2)
        threading.Thread(target=play_sound, args=(1200, 0.1, 0.5), daemon=True).start()
        print("Double left click")
    
    def right_click():
        mouse.click(Button.right)
        threading.Thread(target=play_sound, args=(800, 0.1, 0.5), daemon=True).start()
        print("Right click")
    
    def toggle_right_click_hold():
        nonlocal right_button_held
        if not right_button_held:
            mouse.press(Button.right)
            right_button_held = True
            threading.Thread(target=play_sound, args=(600, 0.2, 0.5), daemon=True).start()
            print("Right button pressed and HELD")
        else:
            mouse.release(Button.right)
            right_button_held = False
            threading.Thread(target=play_sound, args=(600, 0.1, 0.3), daemon=True).start()
            print("Right button RELEASED")
    
    # Initialize filter state for lfilter
    zi = np.zeros(max(len(a), len(b)) - 1)
    
    print(f"Audio click detection active with mean threshold: {MEAN_THRESHOLD}")
    print(f"1 window above threshold: Left Click")
    print(f"2 consecutive windows: Double Left Click")
    print(f"3 consecutive windows: Right Click")
    print(f"4+ consecutive windows: Toggle Right Button Hold")
    print("Listening for high-frequency sounds (7-8 kHz)...")
    
    # For tracking sound on/off transitions
    sound_on = False
    action_performed = False
    
    while not stop_event.is_set():
        try:
            # Get audio data from input queue
            try:
                task = input_queue.get(timeout=0.005)
            except queue.Empty:
                # If we have sound_on but no new data, check if we should end the current sound
                if sound_on and time.time() - last_click_time > 0.1:
                    sound_on = False
                    consecutive_windows = 0
                continue
            
            if isinstance(task, np.ndarray):
                # Apply bandpass filter
                filtered_audio, zi = lfilter(b, a, task, zi=zi)
                
                # Update the context buffer
                if len(filtered_audio) < buffer_size:
                    context_buffer = np.roll(context_buffer, -len(filtered_audio))
                    context_buffer[-len(filtered_audio):] = filtered_audio
                else:
                    context_buffer = filtered_audio[-buffer_size:]
                
                # Check if we're above threshold
                above_threshold, mean_value = is_above_threshold(context_buffer)
                
                now = time.time()
                
                # Logic for tracking consecutive windows and performing actions
                if above_threshold:
                    if not sound_on:
                        # New sound started
                        sound_on = True
                        consecutive_windows = 1
                        action_performed = False
                        last_click_time = now
                    else:
                        # Continuing sound
                        if now - last_click_time > WINDOW_DURATION:
                            consecutive_windows += 1
                            last_click_time = now
                else:
                    # Sound has ended, perform action if not already done
                    if sound_on and not action_performed and consecutive_windows > 0:
                        print("consecutive_windows: ", consecutive_windows)
                        # Perform the appropriate action based on consecutive windows
                        if consecutive_windows == 1:
                            left_click()
                        elif consecutive_windows == 2:
                            left_double_click()
                        elif consecutive_windows == 3:
                            right_click()
                        elif consecutive_windows >= 4:
                            toggle_right_click_hold()
                                
                        action_performed = True
                        click_count += 1
                            
                    sound_on = False
                    consecutive_windows = 0
                
                # Put filtered data in output queue
                output_queue.put(filtered_audio)
            else:
                # Pass through non-numpy arrays unchanged
                output_queue.put(task)
                
            input_queue.task_done()
            
        except Exception as e:
            print(f"Error in audio bandpass filter: {str(e)}")
    
    # Make sure to release the right button if it's held when stopping
    if right_button_held:
        mouse.release(Button.right)
        print("Right button released (cleanup)")
    
    print("audio_bandpass_high_F_filter_stopped")

def audio_bandpass_high_F_filter_durations2(input_queue, output_queue, stop_event, sound_file=None):    
    # Parameters
    SAMPLE_RATE = 16500  # Sampling rate (Hz)
    LOWCUT = 7000.0      # Low-frequency cutoff (Hz)
    HIGHCUT = 8000.0     # High-frequency cutoff (Hz)
    FILTER_ORDER = 5
    
    # Detection parameters
    WINDOW_DURATION = 0.4  # Length of the window (seconds)
    MEAN_THRESHOLD = 200    # Mean of absolute values must exceed this
    WINDOW_SIZE = int(WINDOW_DURATION * SAMPLE_RATE)
    
    # For tracking consecutive windows above threshold
    consecutive_windows = 0
    right_button_held = False
    left_button_held = False
    
    # Pre-compute filter coefficients
    nyquist = 0.5 * SAMPLE_RATE
    low = LOWCUT / nyquist
    high = HIGHCUT / nyquist
    b, a = butter(FILTER_ORDER, [low, high], btype='band')
    
    # Buffer to maintain context for detection
    buffer_duration = 0.1  # seconds
    buffer_size = int(buffer_duration * SAMPLE_RATE)
    context_buffer = np.zeros(buffer_size)
    
    # Load sound file if provided
    try:
        if sound_file:
            click_sound_data, click_sound_samplerate = sf.read(sound_file)
            print(f"Loaded sound file: {sound_file}")
        else:
            click_sound_data = None
    except Exception as e:
        print(f"Warning: Could not load sound file ({e}). Will use default sounds.")
        click_sound_data = None
    
    # Functions to play different sounds without threading
    def play_sound(frequency=1000, duration=0.1, volume=0.5):
        try:
            if click_sound_data is not None:
                sd.play(click_sound_data, click_sound_samplerate)
            else:
                # Generate a simple beep
                # The 'duration' parameter here controls how long the sound plays
                t = np.linspace(0, duration, int(duration * 44100), False)
                beep = volume * np.sin(2 * np.pi * frequency * t)
                sd.play(beep, 44100)
        except Exception as e:
            print(f"Error playing sound: {e}")
    
    # Function to check if mean is above threshold
    def is_above_threshold(signal):
        if len(signal) < WINDOW_SIZE:
            window = signal
        else:
            window = signal[-WINDOW_SIZE:]
        
        mean_abs = np.mean(np.abs(window))
        return mean_abs > MEAN_THRESHOLD, mean_abs
    
    # Mouse control functions
    mouse = Controller()
    
    def left_click():
        nonlocal left_button_held
        nonlocal right_button_held
        left_button_held = False
        right_button_held = False
        mouse.click(Button.left)
        play_sound(1200, 0.1, 0.5)
        print("Left click")
        
    
    def left_double_click():
        nonlocal left_button_held
        nonlocal right_button_held
        left_button_held = False
        right_button_held = False
        mouse.click(Button.left, 2)
        play_sound(1600, 0.1, 0.5)
        print("Double left click")
    
    def right_click():
        nonlocal left_button_held
        nonlocal right_button_held
        left_button_held = False
        right_button_held = False
        mouse.click(Button.right)
        play_sound(800, 0.1, 0.5)
        print("Right click")
    
    def toggle_right_click_hold():
        nonlocal right_button_held
        if not right_button_held:
            mouse.press(Button.right)
            right_button_held = True
            play_sound(600, 0.4, 0.5)  # Longer sound (0.2s) for press
            print("Right button pressed and HELD")
        else:
            mouse.release(Button.right)
            right_button_held = False
            play_sound(600, 0.3, 0.3)  # Shorter sound (0.1s) for release
            print("Right button RELEASED")

    def toggle_left_click_hold():
        nonlocal left_button_held
        if not left_button_held:
            mouse.press(Button.left)
            left_button_held = True
            play_sound(600, 0.4, 0.5)
            print("Left button pressed and HELD")
        else:
            mouse.release(Button.left)
            left_button_held = False
            play_sound(600, 0.3, 0.3)
            print("Left button RELEASED")
    
    # Initialize filter state for lfilter
    zi = np.zeros(max(len(a), len(b)) - 1)
    
    print(f"Audio click detection active with mean threshold: {MEAN_THRESHOLD}")
    print(f"1 window above threshold: Left Click")
    print(f"2 consecutive windows: Double Left Click")
    print(f"3 consecutive windows: Right Click")
    print(f"4+ consecutive windows: Toggle Right Button Hold")
    print("Listening for high-frequency sounds (7-8 kHz)...")
    
    # For improved sound duration tracking
    sound_start_time = None
    above_threshold_last_time = 0
    current_mean = 0
    
    # State machine variables
    waiting_for_sound = True  # Initial state: waiting for sound to begin
    sound_in_progress = False
    
    # Minimum time (seconds) sound must be below threshold to consider it ended
    SOUND_END_GAP = 0.1
    
    # Wait time between consecutive windows for counting
    WINDOW_COUNT_INTERVAL = WINDOW_DURATION * 0.75  # No overlap
    
    while not stop_event.is_set():
        try:
            # Get audio data from input queue
            try:
                task = input_queue.get(timeout=0.005)
            except queue.Empty:
                # Check if we need to handle a sound ending
                now = time.time()
                if sound_in_progress and (now - above_threshold_last_time) > SOUND_END_GAP:
                    # Sound has been below threshold long enough - consider it ended
                    print(f"Sound ended. Duration: {above_threshold_last_time - sound_start_time:.2f}s, Count: {consecutive_windows}")
                    
                    # Perform action based on window count
                    if consecutive_windows == 1:
                        left_click()
                    elif consecutive_windows == 2:
                        left_double_click()
                    elif consecutive_windows == 3:
                        right_click()
                    elif consecutive_windows >= 4:
                        toggle_left_click_hold()
                    
                    # Reset state for next sound
                    sound_in_progress = False
                    waiting_for_sound = True
                    consecutive_windows = 0
                
                continue
            
            if isinstance(task, np.ndarray):
                # Apply bandpass filter
                filtered_audio, zi = lfilter(b, a, task, zi=zi)
                
                # Update the context buffer
                if len(filtered_audio) < buffer_size:
                    context_buffer = np.roll(context_buffer, -len(filtered_audio))
                    context_buffer[-len(filtered_audio):] = filtered_audio
                else:
                    context_buffer = filtered_audio[-buffer_size:]
                
                # Check if we're above threshold
                above_threshold, mean_value = is_above_threshold(context_buffer)
                current_mean = mean_value  # Store for debugging if needed
                
                now = time.time()
                
                # Improved state machine for sound detection
                if above_threshold:
                    above_threshold_last_time = now
                    
                    if waiting_for_sound:
                        # Start of a new sound
                        sound_start_time = now
                        consecutive_windows = 1
                        sound_in_progress = True
                        waiting_for_sound = False
                        print(f"Sound started. Mean: {mean_value:.1f}")
                    
                    elif sound_in_progress:
                        # Sound continuing - check if we should count another window
                        sound_duration = now - sound_start_time
                        
                        # Calculate expected window count based on duration
                        expected_windows = int(sound_duration / WINDOW_COUNT_INTERVAL) + 1
                        
                        # Update window count if needed
                        if expected_windows > consecutive_windows:
                            consecutive_windows = expected_windows
                            print(f"Window count increased to {consecutive_windows}")
                
                # Put filtered data in output queue
                output_queue.put(filtered_audio)
            else:
                # Pass through non-numpy arrays unchanged
                output_queue.put(task)
                
            input_queue.task_done()
            
        except Exception as e:
            print(f"Error in audio bandpass filter: {str(e)}")
    
    # Make sure to release the right button if it's held when stopping
    if right_button_held:
        mouse.release(Button.right)
        print("Right button released (cleanup)")
    
    print("audio_bandpass_high_F_filter_stopped")


def audio_bandpass_high_F_filter_durations3(input_queue, output_queue, stop_event, sound_file=None):    
    # Parameters
    SAMPLE_RATE = 16500  # Sampling rate (Hz)
    LOWCUT = 7000.0      # Low-frequency cutoff (Hz)
    HIGHCUT = 8000.0     # High-frequency cutoff (Hz)
    FILTER_ORDER = 5
    
    # Detection parameters
    WINDOW_DURATION = 0.3  # Length of the window (seconds)
    MEAN_THRESHOLD = 190    # Mean of absolute values must exceed this
    WINDOW_SIZE = int(WINDOW_DURATION * SAMPLE_RATE)
    
    # For tracking consecutive windows above threshold
    consecutive_windows = 0
    right_button_held = False
    left_button_held = False
    
    # Pre-compute filter coefficients
    nyquist = 0.5 * SAMPLE_RATE
    low = LOWCUT / nyquist
    high = HIGHCUT / nyquist
    b, a = butter(FILTER_ORDER, [low, high], btype='band')
    
    # Buffer to maintain context for detection
    buffer_duration = 0.1  # seconds
    buffer_size = int(buffer_duration * SAMPLE_RATE)
    context_buffer = np.zeros(buffer_size)
    
    # Load sound file if provided
    try:
        if sound_file:
            click_sound_data, click_sound_samplerate = sf.read(sound_file)
            print(f"Loaded sound file: {sound_file}")
        else:
            click_sound_data = None
    except Exception as e:
        print(f"Warning: Could not load sound file ({e}). Will use default sounds.")
        click_sound_data = None
    
    # Functions to play different sounds without threading
    def play_sound(frequency=1000, duration=0.1, volume=0.5):
        try:
            if click_sound_data is not None:
                sd.play(click_sound_data, click_sound_samplerate)
            else:
                # Generate a simple beep
                t = np.linspace(0, duration, int(duration * 44100), False)
                beep = volume * np.sin(2 * np.pi * frequency * t)
                sd.play(beep, 44100)
        except Exception as e:
            print(f"Error playing sound: {e}")
    
    # Function to check if mean is above threshold
    def is_above_threshold(signal):
        if len(signal) < WINDOW_SIZE:
            window = signal
        else:
            window = signal[-WINDOW_SIZE:]
        
        mean_abs = np.mean(np.abs(window))
        return mean_abs > MEAN_THRESHOLD, mean_abs
    
    # Mouse control functions
    mouse = Controller()
    
    def left_click():
        nonlocal left_button_held
        nonlocal right_button_held
        left_button_held = False
        right_button_held = False
        mouse.click(Button.left)
        play_sound(1200, 0.1, 0.5)
        print("Left click")
        
    def left_double_click():
        nonlocal left_button_held
        nonlocal right_button_held
        left_button_held = False
        right_button_held = False
        mouse.click(Button.left, 2)
        play_sound(1600, 0.1, 0.5)
        print("Double left click")
    
    def right_click():
        nonlocal left_button_held
        nonlocal right_button_held
        left_button_held = False
        right_button_held = False
        mouse.click(Button.right)
        play_sound(800, 0.1, 0.5)
        print("Right click")
    
    def toggle_right_click_hold():
        nonlocal right_button_held
        if not right_button_held:
            mouse.press(Button.right)
            right_button_held = True
            play_sound(600, 0.4, 0.5)
            print("Right button pressed and HELD")
        else:
            mouse.release(Button.right)
            right_button_held = False
            play_sound(600, 0.3, 0.3)
            print("Right button RELEASED")

    def toggle_left_click_hold():
        nonlocal left_button_held
        if not left_button_held:
            mouse.press(Button.left)
            left_button_held = True
            play_sound(600, 0.4, 0.5)
            print("Left button pressed and HELD")
        else:
            mouse.release(Button.left)
            left_button_held = False
            play_sound(600, 0.3, 0.3)
            print("Left button RELEASED")
    
    # Initialize filter state for lfilter
    zi = np.zeros(max(len(a), len(b)) - 1)
    
    print(f"Audio click detection active with mean threshold: {MEAN_THRESHOLD}")
    print(f"1 window above threshold: Left Click")
    print(f"2 consecutive windows: Double Left Click")
    print(f"3 consecutive windows: Right Click")
    print(f"4+ consecutive windows: Toggle Left Button Hold")
    print("Listening for high-frequency sounds (7-8 kHz)...")
    
    # State machine variables - simplified direct window approach
    currently_above_threshold = False
    window_count = 0
    
    # Time to wait before checking for next window
    WINDOW_CHECK_INTERVAL = WINDOW_DURATION * 1 
    last_window_check_time = 0
    
    # Gap time to wait for signal to go below threshold before starting a new sequence
    GAP_AFTER_ACTION = 0.4  # seconds to wait after performing an action
    last_action_time = 0
    
    while not stop_event.is_set():
        try:
            # Get audio data from input queue
            try:
                task = input_queue.get(timeout=0.005)
            except queue.Empty:
                # If we're counting windows but signal stopped, finalize the action
                now = time.time()
                if window_count > 0 and now - last_window_check_time > WINDOW_CHECK_INTERVAL * 2:
                    print(f"Signal ended after {window_count} windows")
                    
                    # Perform action based on window count
                    if window_count == 1:
                        left_click()
                    elif window_count == 2:
                        left_double_click()
                    elif window_count == 3:
                        right_click()
                    elif window_count >= 4:
                        toggle_left_click_hold()
                    
                    # Reset window count
                    window_count = 0
                    last_action_time = now
                
                continue
            
            if isinstance(task, np.ndarray):
                # Apply bandpass filter
                filtered_audio, zi = lfilter(b, a, task, zi=zi)
                
                # Update the context buffer
                if len(filtered_audio) < buffer_size:
                    context_buffer = np.roll(context_buffer, -len(filtered_audio))
                    context_buffer[-len(filtered_audio):] = filtered_audio
                else:
                    context_buffer = filtered_audio[-buffer_size:]
                
                # Check if we're above threshold
                above_threshold, mean_value = is_above_threshold(context_buffer)
                
                now = time.time()
                
                # Only process if enough time has passed since the last action
                if now - last_action_time > GAP_AFTER_ACTION:
                    # Time-based window checking
                    if now - last_window_check_time > WINDOW_CHECK_INTERVAL:
                        last_window_check_time = now
                        
                        if above_threshold:
                            # We found a window above threshold
                            if window_count == 0:
                                # First window above threshold
                                print(f"First window above threshold. Mean: {mean_value:.1f}")
                            else:
                                # Another consecutive window above threshold
                                print(f"Window {window_count+1} above threshold. Mean: {mean_value:.1f}")
                            
                            # Increment window count
                            window_count += 1
                        else:
                            # We found a window below threshold
                            if window_count > 0:
                                # If we had at least one window above threshold, perform action
                                print(f"Signal went below threshold after {window_count} windows")
                                
                                # Perform action based on window count
                                if window_count == 1:
                                    left_click()
                                elif window_count == 2:
                                    left_double_click()
                                elif window_count == 3:
                                    right_click()
                                elif window_count >= 4:
                                    toggle_left_click_hold()
                                
                                # Reset window count
                                window_count = 0
                                last_action_time = now
                
                # Put filtered data in output queue
                output_queue.put(filtered_audio)
            else:
                # Pass through non-numpy arrays unchanged
                output_queue.put(task)
                
            input_queue.task_done()
            
        except Exception as e:
            print(f"Error in audio bandpass filter: {str(e)}")
    
    # Make sure to release buttons if they're held when stopping
    if right_button_held:
        mouse.release(Button.right)
        print("Right button released (cleanup)")
    if left_button_held:
        mouse.release(Button.left)
        print("Left button released (cleanup)")
    
    print("audio_bandpass_high_F_filter_stopped")


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
    gyr_alpha = 0.00 # 0.05
    acc_alpha = 0.000 # 0.005
    salpha = 0.00 # Originally 0.65, 0.75 for demo
    flg = 0
    longcounter = 0 # Counter to add data into csv
    
    gyr_prev = []
    acc_prev = []
    gyr_cur = []
    acc_cur = []

    Qold = np.zeros((1, 4))
    Qnew = np.zeros((1, 4))
    Qoldek = np.zeros((1, 4))
    Qnewek = np.zeros((1, 4))
    r2d = 57.2958

    maxavg = 50 # Max array size. Essential tremors 4-8Hz
    movingavg = np.zeros((maxavg,3)) # Array for moving average 
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
    b, a = signal.ellip(3, 0.1, 60, [1.7,22], btype='bandstop', fs=100) # bandstop

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
            Qoldek = Qold # For standard EKF
            #ekf = EKF(frequency=17.3,frame='ENU',q0=Qold,noises=[0.1**2, 0.15**4, 0.8**2]) # EKF init from old code 0.6**2 // Was 0.1, 0.15 before \/
            rekf = rek.OREKF(frequency=95,frame='ENU',q0=Qold,noises=[2.5**4, 2.6**4, 0.8**2]) # Initialise EKF function. Noise = [Process covariance, Measurement covariance]
            ekf = EKF(frequency=95,frame='ENU',q0=Qold,noises=[2.5**4, 2.6**4, 0.8**2])
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
            gyroraw, accraw = input_queue.get(timeout = 0.01)
            gyro = np.array([gyroraw],dtype="float")
            acc = np.array([accraw],dtype="float")
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
            gyrochange = movingavg.max(axis=0) - movingavg.min(axis=0)
            #print(gyrochange)
            
            # if(any(i < 1 for i in gyrochange) == True): # Go through gyro XYZ and check if change < 1 degree 
            #     gyro = np.full((3, 1), 0)
            #     print("Stationary! Let's stop drifting...")

            drifti = np.argwhere(gyrochange < 0.05)
            if drifti.size != 0:
                for x in np.nditer(drifti):
                    gyro[0,x] = 0
            # print("stationary")
            # print(gyro)

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
            Qnew = rekf.rupdate(Qold, gyro[0], acc_cur[0]) # Update EKF function. See: https://github.com/Mayitzin/ahrs/blob/master/ahrs/filters/ekf.py#L1336
            Qold = Qnew
            Qnewek = ekf.update(Qoldek, gyr_cur[0], acc_cur[0])
            Qoldek = Qnewek

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
            filangles[1], zp  = signal.lfilter(b, a, [pitch], zi=zp)
            filangles[2], zy  = signal.lfilter(b, a, [yaw], zi=zy)
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

            ##### EKF angles str #####
            ekfangles = q2rpy(Qnewek)*r2d # Get euler angles from quaternions
            ##### EKF angles end #####            

            # fields=[angles[0],angles[1],angles[2],filangles[0],filangles[1],filangles[2],ekfangles[0],ekfangles[1],ekfangles[2]]
            # with open(r'TREKfoo.csv', 'a') as f:
            #         writer = csv.writer(f)
            #         writer.writerow(fields)

            # longcounter = longcounter + 1
            # if(longcounter >= 95):
            #     with open(r'TREKfoo.csv', 'a') as f:
            #         writer = csv.writer(f)
            #         writer.writerow(fields)
            #     longcounter = 0

            #output_queue.put((angles[0],angles[1],angles[2]))
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

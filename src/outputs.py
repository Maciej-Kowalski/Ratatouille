import queue
import threading
import time
import numpy as np
from pynput.mouse import Button, Controller
from screeninfo import get_monitors
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import csv
import os


def no_output(input_queue, stop_event, _):
    while not stop_event.is_set():
        try:
            audio = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
        except queue.Empty:
            continue
    print("no_output stopped")
def print_continuous_audio(input_queue, stop_event, _):
    while not stop_event.is_set():
        try:
            audio = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f'{audio[0]:+6.4f}')
        except queue.Empty:
            continue
    print("print_continuous_aud stopped")
def print_raw_IMU(input_queue, stop_event):
    while not stop_event.is_set():
        try:
            gyro, acc = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f"gyro: {[f'{x:8.4f}' for x in gyro]}, acc: {[f'{x:8.4f}' for x in acc]}")
            input_queue.task_done()
        except queue.Empty:
            continue
    print("raw_IMU_stopped")

def print_comp_IMU(input_queue, stop_event):
    t = time.perf_counter()
    while not stop_event.is_set():
        try:
            pitch, roll = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            if time.perf_counter()-t > 0.05:
                print(f"pitch_est: {pitch:8.4f}, roll_est: {roll:8.4f}")
                t = time.perf_counter()
            input_queue.task_done()
        except queue.Empty:
            continue
    print("comp_IMU_stopped")

def print_EK_filter_IMU(input_queue, stop_event):
    while not stop_event.is_set():
        try:
            roll, pitch, yaw = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f"roll: {roll:8.4f}, pitch: {pitch:8.4f}, yaw: {yaw:8.4f}")
            input_queue.task_done()
        except queue.Empty:
            continue
    print("EKF_IMU_stopped")

def mouse_cursor_mapping_no_cal(input_queue, stop_event):
    mouse = Controller()
    screen = get_monitors()
    while not stop_event.is_set():
        try:
            roll, pitch, yaw = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            x = remap(yaw, (0,screen[0].width),(-60,20))
            y = remap(pitch, (screen[0].height,0),(-50,50))
            print(f"roll: {roll:8.4f}, pitch: {pitch:8.4f}, yaw: {yaw:8.4f}, x: {x}, y: {y}")
            mouse.position = (x, y)
            input_queue.task_done()
        except queue.Empty:
            continue
    print("cursor_mouse_stopped")

def mouse_cursor_mapping_self_cal(input_queue, stop_event):
    mouse = Controller()
    screen = get_monitors()
    t = time.perf_counter()
    while not stop_event.is_set():
        try:
            roll, pitch, yaw = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f"roll: {roll:8.4f}, pitch: {pitch:8.4f}, yaw: {yaw:8.4f}")
            if time.perf_counter - t > 10:
                x = remap(yaw, (0,screen[0].width),(-60,20))
                y = remap(pitch, (screen[0].height,0),(-50,50))
                mouse.position = (x, y)
            input_queue.task_done()
        except queue.Empty:
            continue
    print("cursor_mouse_stopped")

def mouse_cursor_mapping(input_queue, stop_event):
    mouse = Controller()
    screen = get_monitors()
    range_horizontal, range_vertical = calibrate(input_queue)
    while not stop_event.is_set():
        try:
            roll, pitch, yaw = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f"roll: {roll:8.4f}, pitch: {pitch:8.4f}, yaw: {yaw:8.4f}")
            x = remap(yaw, (0,screen[0].width),range_horizontal)
            y = remap(pitch, (0,screen[0].height),range_vertical)
            mouse.position = (x, y)
            input_queue.task_done()
        except queue.Empty:
            continue
    print("cursor_mouse_stopped")

def calibrate(input_queue):
    """Perform calibration by waiting for user input and retrieving data."""
    print("Calibration started. After following each instruction wait for ~0.5s and press enter")
    message = [
        "Look at the middle of the screen",
        "Look to the centre of the left edge",
        "Look to the centre of the right edge",
        "Look to the centre of the top edge",
        "Look to the centre of the bottom edge",
    ]
    calibration_count = 5
    cal_data = []
    for i in range(calibration_count):
        # Wait for user signal (Enter key press)
        input(message[i])

        # Get the most recent item from the queue
        try:
            while not input_queue.empty():
                data = input_queue.get_nowait()  # Remove all old data
            cal_data.append(data)  # Store the latest item
            print(data)
        except queue.Empty:
            print("No data in queue to calibrate.")

    range_horizontal = (int(cal_data[1][2]),int(cal_data[2][2]))
    range_vertical = (int(cal_data[3][1]),int(cal_data[4][1]))
    print(f"hor: {range_horizontal}, ver: {range_vertical}, middle: {cal_data[0][2]}, {cal_data[0][1]}")
    time.sleep(3)
    print("Calibration complete. Starting normal operation.")

    with input_queue.mutex:
        input_queue.queue.clear()
    return range_horizontal, range_vertical

def remap(value, new_range, old_range):
    if old_range[1] > old_range[0]:
        if value < old_range[0]:
            value = old_range[0]
        elif value > old_range[1]:
            value = old_range[1]
    else:
        if value < old_range[1]:
            value = old_range[1]
        elif value > old_range[0]:
            value = old_range[0]
    return int((new_range[1] - new_range[0])*(value - old_range[0]) / (old_range[1] - old_range[0]) + new_range[0])

def print_raw_audio(input_queue, stop_event):
    while not stop_event.is_set():
        try:
            audio1, audio2 = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f"audio1: {audio1:4d}, audio2: {audio2:4d}")
            input_queue.task_done()
        except queue.Empty:
            continue
    print("raw_audio_stopped")

def print_counter_audio(input_queue, stop_event):
    counter = 0
    t = time.perf_counter()
    while not stop_event.is_set():
        try:
            audio = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            counter += 1
            #print(f"counter: {counter:4d}, audio1: {audio1:4d}, audio2: {audio2:4d}")
            if (counter % 1000 == 0):
                print(f"counter: {counter}, audio: {audio} time: {time.perf_counter()-t}")
                t = time.perf_counter()
            input_queue.task_done()
        except queue.Empty:
            #print("Queue empty")
            continue
    print("print_counter_audio_stopped")

def print_counter_audio_buffered(input_queue, stop_event, buffer_size):
    counter = 0
    t = time.perf_counter()
    while not stop_event.is_set():
        try:
            audio = input_queue.get(timeout = 0.1)  # Use timeout to prevent hanging
            #print(f"counter: {counter:4d}, audio1: {audio1:4d}, audio2: {audio2:4d}")
            counter += 1*buffer_size
            if counter % 100 == 0:
                queue_length = input_queue.qsize()
                print(f"counter: {counter}, audio: {audio} time: {time.perf_counter()-t:8.4f}, queue: {queue_length} ")
                t = time.perf_counter()
            input_queue.task_done()
        except queue.Empty:
            #print("Empty")
            continue
    print("print_counter_audio_buffered_stopped")

import time
import queue
import csv

def plot_audio_buffered(input_queue, stop_event, buffer_size, plot_interval=100, max_points=5000):
    # Set up the figure and axis
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(10, 5))
    
    # Configure plot for audio waveform only
    ax.set_title('Live Audio Waveform')
    ax.set_xlabel('Sample Number')
    ax.set_ylabel('Amplitude')
    ax.grid(True, alpha=0.3)
    
    # Create the waveform line
    waveform_line, = ax.plot([], [], 'b-', linewidth=0.5)
    
    # Initialize data storage
    all_samples = np.array([], dtype=np.int16)
    
    counter = 0
    t_start = time.perf_counter()
    
    try:
        while not stop_event.is_set():
            try:
                # Get audio buffer from queue with timeout
                audio_buffer = input_queue.get(timeout=0.1)
                counter += buffer_size
                
                # Convert to numpy array if it isn't already
                if not isinstance(audio_buffer, np.ndarray):
                    audio_buffer = np.array(audio_buffer)
                
                # Append new samples to our record
                all_samples = np.append(all_samples, audio_buffer)
                
                # Limit the number of points to avoid memory issues
                if len(all_samples) > max_points:
                    all_samples = all_samples[-max_points:]
                
                # Process at regular intervals
                if counter % plot_interval == 0:
                    # Update the plot
                    x_data = np.arange(len(all_samples))
                    waveform_line.set_data(x_data, all_samples)
                    
                    # Adjust axis limits
                    ax.set_xlim(max(0, len(all_samples) - max_points), len(all_samples))
                    
                    # Find min/max of visible data for y-axis
                    visible_data = all_samples[-max_points:] if len(all_samples) > max_points else all_samples
                    y_min = np.min(visible_data) if len(visible_data) > 0 else -1
                    y_max = np.max(visible_data) if len(visible_data) > 0 else 1
                    margin = (y_max - y_min) * 0.1 if y_max > y_min else 10
                    ax.set_ylim(y_min - margin, y_max + margin)
                    
                    # Update title with sample count
                    elapsed = time.perf_counter() - t_start
                    ax.set_title(f'Live Audio Waveform - Samples: {len(all_samples):,}, Time: {elapsed:.2f}s')
                    
                    # Refresh the plot
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                
                # Mark as done
                input_queue.task_done()
            
            except queue.Empty:
                # Just continue if queue is empty
                continue
    
    except KeyboardInterrupt:
        print("Plot interrupted by user")
    
    finally:
        print("plot_audio_buffered stopped")
        plt.ioff()
        plt.close(fig)


def plot_continuous_audio(input_queue, stop_event, _):
    # Plot parameters
    BUFFER_SIZE = 5000  # Number of samples to display
    UPDATE_INTERVAL = 100  # Update plot every 50ms
    
    # Initialize plot
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(10, 5))
    
    # Create empty line
    line, = ax.plot([], [], 'b-', linewidth=0.8)
    
    # Configure plot
    ax.set_title('Live Audio Waveform')
    ax.set_xlabel('Sample Number')
    ax.set_ylabel('Amplitude')
    ax.set_ylim(-500, 4000)  # Assuming normalized audio (-1 to 1)
    ax.grid(True, alpha=0.3)
    
    # Initialize data buffer
    data_buffer = np.zeros(BUFFER_SIZE)
    
    # For tracking performance
    last_update_time = time.time()
    sample_count = 0
    
    try:
        while not stop_event.is_set():
            try:
                # Get audio data from queue
                audio = input_queue.get(timeout=0.01)
                
                # Shift buffer and add new samples
                shift_size = min(len(audio), BUFFER_SIZE)
                data_buffer = np.roll(data_buffer, -shift_size)
                
                # Add new data to the end of the buffer
                if len(audio) >= shift_size:
                    data_buffer[-shift_size:] = audio[-shift_size:]
                else:
                    data_buffer[-len(audio):] = audio
                
                sample_count += len(audio)
                input_queue.task_done()
                
                # Update plot at regular intervals
                current_time = time.time()
                if current_time - last_update_time > UPDATE_INTERVAL/1000:
                    # Update plot data
                    x_data = np.arange(BUFFER_SIZE)
                    line.set_data(x_data, data_buffer)
                    
                    # Adjust plot limits if needed
                    ax.set_xlim(0, BUFFER_SIZE)
                    
                    # Update title with info
                    ax.set_title(f'Live Audio Waveform - Samples: {sample_count}')
                    
                    # Redraw the plot
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                    
                    # Reset timer
                    last_update_time = current_time
                    
            except queue.Empty:
                # # Update plot occasionally even when queue is empty
                # if time.time() - last_update_time > UPDATE_INTERVAL/1000:
                #     fig.canvas.draw_idle()
                #     fig.canvas.flush_events()
                #     last_update_time = time.time()
                continue
                
    except KeyboardInterrupt:
        print("Plot interrupted by user")
    
    finally:
        print("plot_continuous_audio stopped")
        plt.ioff()
        plt.close(fig)


def plot_audio_with_fft(input_queue, stop_event, _):
    import matplotlib.pyplot as plt
    import numpy as np
    import queue
    import time
    
    # Plot parameters
    SAMPLE_RATE = 16500  # Hz (matching your previous code)
    BUFFER_SIZE = 4096   # Number of samples to display (power of 2 for efficient FFT)
    UPDATE_INTERVAL = 100  # Update plot every 50ms
    
    # For time display
    TIME_WINDOW = BUFFER_SIZE / SAMPLE_RATE  # Duration in seconds
    
    # Initialize plot with two subplots
    plt.ion()  # Turn on interactive mode
    fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Create empty lines
    line_time, = ax_time.plot([], [], 'b-', linewidth=0.8)
    line_freq, = ax_freq.plot([], [], 'r-', linewidth=0.8)
    
    # Configure time domain plot
    ax_time.set_title('Live Audio Waveform')
    ax_time.set_xlabel('Time (seconds)')
    ax_time.set_ylabel('Amplitude')
    ax_time.set_ylim(-1000, 5000)
    ax_time.grid(True, alpha=0.3)
    
    # Set x-axis to show time instead of sample numbers
    x_time = np.linspace(0, TIME_WINDOW, BUFFER_SIZE)
    ax_time.set_xlim(0, TIME_WINDOW)
    
    # Configure frequency domain plot
    ax_freq.set_title('Frequency Spectrum')
    ax_freq.set_xlabel('Frequency (Hz)')
    ax_freq.set_ylabel('Magnitude (dB)')
    ax_freq.set_ylim(0, 160)
    ax_freq.grid(True, alpha=0.3)
    
    # Frequency axis setup
    freqs = np.fft.rfftfreq(BUFFER_SIZE, 1/SAMPLE_RATE)
    ax_freq.set_xlim(0, SAMPLE_RATE/2)  # Nyquist frequency
    
    # Add vertical lines for the bandpass region
    LOWCUT, HIGHCUT = 7000.0, 8000.0  # from your previous code
    ax_freq.axvline(x=LOWCUT, color='g', linestyle='--', alpha=0.5, label='Filter Cutoff')
    ax_freq.axvline(x=HIGHCUT, color='g', linestyle='--', alpha=0.5)
    ax_freq.legend(loc='upper right')
    
    # Initialize data buffer with zeros
    data_buffer = np.zeros(BUFFER_SIZE)
    
    # For tracking performance
    last_update_time = time.time()
    sample_count = 0
    
    # Window function for FFT (reduces spectral leakage)
    window = np.hanning(BUFFER_SIZE)
    
    # Adjust layout
    plt.tight_layout()
    
    try:
        while not stop_event.is_set():
            try:
                # Get audio data from queue
                audio = input_queue.get(timeout=0.01)
                
                # Shift buffer and add new samples
                shift_size = min(len(audio), BUFFER_SIZE)
                data_buffer = np.roll(data_buffer, -shift_size)
                
                # Add new data to the end of the buffer
                if len(audio) >= shift_size:
                    data_buffer[-shift_size:] = audio[-shift_size:]
                else:
                    data_buffer[-len(audio):] = audio
                
                sample_count += len(audio)
                input_queue.task_done()
                
                # Update plot at regular intervals
                current_time = time.time()
                if current_time - last_update_time > UPDATE_INTERVAL/1000:
                    # Update time domain plot with proper time axis
                    line_time.set_data(x_time, data_buffer)
                    
                    # Calculate FFT (apply window to reduce spectral leakage)
                    windowed_data = data_buffer * window
                    fft_data = np.fft.rfft(windowed_data)
                    
                    # Convert to magnitude in dB scale (with noise floor)
                    magnitude = np.abs(fft_data)
                    magnitude_db = 20 * np.log10(magnitude + 1e-10)  # Add small value to avoid log(0)
                    
                    # Update frequency domain plot
                    line_freq.set_data(freqs, magnitude_db)
                    
                    # Dynamically adjust y-axis for time domain
                    data_min = np.min(data_buffer)
                    data_max = np.max(data_buffer)
                    margin = max(0.1, (data_max - data_min) * 0.1)
                    ax_time.set_ylim(data_min - margin, data_max + margin)
                    
                    # Update titles with info
                    ax_time.set_title(f'Live Audio Waveform ({TIME_WINDOW:.2f} sec window) - Total Samples: {sample_count}')
                    
                    # Redraw the plots
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                    
                    # Reset timer
                    last_update_time = current_time
                    
            except queue.Empty:
                # Update plot occasionally even when queue is empty
                if time.time() - last_update_time > UPDATE_INTERVAL/1000:
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                    last_update_time = time.time()
                continue
                
    except KeyboardInterrupt:
        print("Plot interrupted by user")
    
    finally:
        print("plot_audio_with_fft stopped")
        plt.ioff()
        plt.close(fig)


def record_and_plot_audio(output_queue, stop_event, filename=None):
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy import signal
    import queue
    import time
    import os
    
    # Handle the filename parameter correctly
    if filename is None or not isinstance(filename, (str, bytes, os.PathLike)):
        # Default filename if none provided or invalid type
        filename = "recorded_audio.npy"
    
    # Parameters
    RECORD_DURATION = 10  # seconds
    SAMPLE_RATE = 16500   # Hz
    
    # For fixed-time recording approach
    start_time = time.time()
    end_time = start_time + RECORD_DURATION
    
    # Initialize buffer for audio collection
    audio_buffer = []  # Start with a list for faster appends
    
    print(f"Starting audio recording for {RECORD_DURATION} seconds...")
    print(f"Output will be saved to: {filename}")
    last_update_time = start_time
    
    try:
        # Record until time is up (fixed duration approach)
        while time.time() < end_time and not stop_event.is_set():
            try:
                # Get audio data from queue with minimal timeout
                audio_chunk = output_queue.get(timeout=0.001)
                
                # Append to our recording buffer (using list for efficiency)
                audio_buffer.append(audio_chunk)
                
                # Mark as done immediately to avoid queue backup
                output_queue.task_done()
                
                # Update display at intervals
                current_time = time.time()
                if current_time - last_update_time >= 0.5:  # Less frequent updates
                    elapsed = current_time - start_time
                    remaining = max(0, RECORD_DURATION - elapsed)
                    
                    # Estimate progress based on time rather than samples
                    time_progress = min(100, (elapsed / RECORD_DURATION) * 100)
                    
                    print(f"\rRecording: {time_progress:.1f}% (time-based) | Elapsed: {elapsed:.1f}s | Remaining: {remaining:.1f}s | Chunks: {len(audio_buffer)}", end="")
                    last_update_time = current_time
                
            except queue.Empty:
                # Very short timeout and continue immediately
                continue
        
        print("\nProcessing recorded audio...")
        
        # Convert list of arrays to a single flat array
        if audio_buffer:
            try:
                # Handle different input array shapes
                if isinstance(audio_buffer[0], np.ndarray) and audio_buffer[0].size > 1:
                    # If chunks are arrays, concatenate them
                    full_audio = np.concatenate(audio_buffer)
                else:
                    # If chunks are scalars or single-element arrays
                    full_audio = np.array(audio_buffer).flatten()
                
                # Save to file
                np.save(filename, full_audio)
                
                actual_duration = len(full_audio) / SAMPLE_RATE
                print(f"Recording complete! Saved {len(full_audio)} samples ({actual_duration:.2f} seconds) to {filename}")
                
                # Simple analysis first
                print("Generating basic visualization...")
                
                # Create a simpler figure (for speed)
                plt.figure(figsize=(10, 6))
                
                # Plot time domain only first (faster)
                time_axis = np.linspace(0, actual_duration, len(full_audio))
                plt.plot(time_axis, full_audio, 'b-', linewidth=0.5)
                plt.title(f'Recorded Audio Waveform ({actual_duration:.2f} seconds)')
                plt.xlabel('Time (seconds)')
                plt.ylabel('Amplitude')
                plt.grid(True, alpha=0.3)
                
                plt.tight_layout()
                plt.show(block=False)  # Non-blocking to continue processing
                
                print("Basic visualization complete. Generating detailed analysis...")
                
                # Create more detailed plots after showing the basic one
                fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(12, 8))
                
                # Time domain
                ax_time.plot(time_axis, full_audio, 'b-', linewidth=0.5)
                ax_time.set_title(f'Recorded Audio Waveform ({actual_duration:.2f} seconds)')
                ax_time.set_xlabel('Time (seconds)')
                ax_time.set_ylabel('Amplitude')
                ax_time.grid(True, alpha=0.3)
                
                # Compute the spectrogram with reduced complexity
                # Use larger window size and less overlap for faster computation
                f, t, Sxx = signal.spectrogram(full_audio, fs=SAMPLE_RATE, 
                                              nperseg=2048, noverlap=512)
                
                # Plot spectrogram
                spec = ax_freq.pcolormesh(t, f, 10 * np.log10(Sxx + 1e-10), 
                                         shading='gouraud', cmap='viridis')
                ax_freq.set_title('Spectrogram (Frequency vs Time)')
                ax_freq.set_ylabel('Frequency (Hz)')
                ax_freq.set_xlabel('Time (seconds)')
                
                fig.colorbar(spec, ax=ax_freq, label='Power Spectral Density (dB)')
                
                # Add bandpass region
                LOWCUT, HIGHCUT = 7000.0, 8000.0
                ax_freq.axhline(y=LOWCUT, color='r', linestyle='--', alpha=0.7)
                ax_freq.axhline(y=HIGHCUT, color='r', linestyle='--', alpha=0.7)
                
                plt.tight_layout()
                plt.show()
                
            except Exception as e:
                print(f"Error processing audio data: {str(e)}")
        else:
            print("Warning: No audio data was captured!")
            
    except KeyboardInterrupt:
        print("\nRecording interrupted by user")
    
    finally:
        print("record_and_plot_audio function stopped")


def timing_audio_buffered(input_queue, stop_event, buffer_size):
    frequency = input("Frequency: ")
    t_start = time.perf_counter()
    t_last_packet = t_start
    raw_data = []  # Array to store all raw data points
    time_differences = []
    packets_received = 0
    first_packet_first_element = None
    hundredth_packet_last_element = None
    sample_counter = 0  # Counter for continuous sample numbering

    while not stop_event.is_set() and packets_received < 100:
        try:
            audio = input_queue.get(timeout=0.1)  # Get audio packet
            packets_received += 1
            
            # Record timing information
            t_now = time.perf_counter()
            if packets_received == 1:
                first_packet_first_element = audio[0]
                t_start = t_now
            if packets_received == 100:
                hundredth_packet_last_element = audio[-1]
            
            if packets_received > 1:
                time_elapsed = t_now - t_last_packet
                time_differences.append(time_elapsed)
            
            # Store ALL data points from the packet with continuous numbering
            for i, sample in enumerate(audio):
                raw_data.append([
                    sample_counter + i,  # Continuous sample number
                    sample,             # Sample value
                    packets_received,   # Packet number
                    t_now - t_start     # Time since start
                ])
            
            sample_counter += len(audio)
            t_last_packet = t_now
            input_queue.task_done()
        except queue.Empty:
            continue

    # Calculate statistics
    if len(time_differences) > 0:
        time_array = np.array(time_differences)
        mean_time = np.mean(time_array)
        std_time = np.std(time_array)
        min_time = np.min(time_array)
        max_time = np.max(time_array)
        percentile_10 = np.percentile(time_array, 10)
        percentile_90 = np.percentile(time_array, 90)
    else:
        mean_time = std_time = min_time = max_time = percentile_10 = percentile_90 = 0

    # Write summary data to timing_data.csv (append mode)
    summary_data = [
        frequency,
        buffer_size,
        first_packet_first_element,
        hundredth_packet_last_element,
        mean_time,
        std_time,
        min_time,
        percentile_10,
        percentile_90,
        max_time
    ]
    
    # Write summary data (append to file)
    file_exists = os.path.isfile('timing_data.csv')
    with open("timing_data.csv", "a", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        if not file_exists:
            csv_writer.writerow([
                "Frequency", "Buffer Size", "First Element (1st packet)", 
                "Last Element (100th packet)", "Mean Time", "Std Dev Time", 
                "Min Time", "10th Percentile", "90th Percentile", "Max Time"
            ])
        csv_writer.writerow(summary_data)

    # Write ALL raw data points to separate file
    raw_filename = f"raw_data_{frequency}_{buffer_size}.csv"
    with open(raw_filename, "w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow([
            "Sample Number", 
            "Sample Value", 
            "Packet Number", 
            "Time Since Start (s)"
        ])
        csv_writer.writerows(raw_data)

    print(f"Data collection complete. Saved {len(raw_data)} samples.")

def test_monitors():
    m = get_monitors()
    print(m[0].width)

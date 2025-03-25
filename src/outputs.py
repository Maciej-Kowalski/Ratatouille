import queue
import threading
import time
import numpy as np
from pynput.mouse import Button, Controller
from screeninfo import get_monitors
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

def plot_audio_buffered(input_queue, stop_event, buffer_size, plot_interval=100, max_points=10000):
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



def timing_audio_buffered(input_queue, stop_event, buffer_size):
    #packet size = input("Packet size: ")
    #frequency = input("Frequency: ")
    first_value = 0
    last_value = 0
    t_start = time.perf_counter()
    t_last_packet = t_start
    data = []  # Array to store results
    first_packet_received = False
    packets_received = 0

    while not stop_event.is_set():
        try:
            audio = input_queue.get(timeout=0.1)  # Use timeout to prevent hanging
            if not first_packet_received:
                first_packet_received = True
                t_start = time.perf_counter()  # Start the 10-second timer
                first_value = audio[0]

            packets_received += 1
            counter += 1 * buffer_size
            t_now = time.perf_counter()
            time_elapsed = t_now - t_last_packet
            #queue_length = input_queue.qsize()

            # Store data in the array
            data.append([counter, audio[0], audio[-1], time_elapsed, queue_length])

            t_last_packet = t_now

            # Check if 10 seconds have passed since the first packet
            if packets_received >= 100:
                break

            input_queue.task_done()
        except queue.Empty:
            continue

    # Save data to a CSV file
    with open("audio_data_3200.csv", "w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["Counter", "First Element", "Last Element", "Time Elapsed", "Queue Size"])  # Header
        csv_writer.writerows(data)  # Write all rows

    print("Done")

def test_monitors():
    m = get_monitors()
    print(m[0].width)

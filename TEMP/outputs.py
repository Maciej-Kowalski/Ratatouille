import queue
import threading
import time
from pynput.mouse import Button, Controller
from screeninfo import get_monitors

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
                print(f"pitch: {pitch:8.4f}, roll: {roll:8.4f}")
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

def mouse_cursor_mapping(input_queue, stop_event):
    mouse = Controller()
    screen = get_monitors()
    while not stop_event.is_set():
        try:
            roll, pitch, yaw = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            x = remap(roll, (0,screen[0].width),(-40,40))
            y = remap(pitch, (0,screen[0].height),(50,-50))
            mouse.position = (x, y)
            input_queue.task_done()
        except queue.Empty:
            continue
    print("cursor_mouse_stopped")

def remap(value, new_range, old_range):
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
            audio1, audio2 = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            counter += 1
            #print(f"counter: {counter:4d}, audio1: {audio1:4d}, audio2: {audio2:4d}")
            if (counter % 10000 == 0):
                print(f"counter: {counter}, audio: {audio1} time: {time.perf_counter()-t}")
                t = time.perf_counter()
            input_queue.task_done()
        except queue.Empty:
            continue
    print("print_counter_audio_stopped")

def print_counter_audio_buffered(input_queue, stop_event, buffer_size):
    counter = 0
    t = time.perf_counter()
    while not stop_event.is_set():
        try:
            task = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            audio1 = task['audio1']  # Retrieve all audio1 values
            audio2 = task['audio2']  # Retrieve all audio2 values
            #print(f"counter: {counter:4d}, audio1: {audio1:4d}, audio2: {audio2:4d}")
            counter += 1*buffer_size
            print(f"counter: {counter}, audio: {audio1[buffer_size-1]} time: {time.perf_counter()-t}")
            t = time.perf_counter()
            input_queue.task_done()
        except queue.Empty:
            continue
    print("print_counter_audio_buffered_stopped")

def test_monitors():
    m = get_monitors()
    print(m[0].width)

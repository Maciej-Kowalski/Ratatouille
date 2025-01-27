def print_raw_IMU(input_queue, stop_event):
    while not stop_event.is_set():
        try:
            gyro, acc = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f"gyro: {[f'{x:8.4f}' for x in gyro]}, acc: {[f'{x:8.4f}' for x in acc]}")
            input_queue.task_done()
        except queue.Empty:
            continue
    print("raw_IMU_stopped")

def print_raw_audio(input_queue, stop_event):
    while not stop_event.is_set():
        try:
            audio1, audio2 = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f"audio1: {audio1:4d}, audio2: {audio2:4d}")
            input_queue.task_done()
        except queue.Empty:
            continue
    print("raw_IMU_stopped")

def print_counter_audio(input_queue, stop_event):
    counter = 1
    while not stop_event.is_set():
        try:
            audio1, audio2 = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f"counter: {counter:4d}, audio1: {audio1:4d}, audio2: {audio2:4d}")
            input_queue.task_done()
        except queue.Empty:
            continue
    print("raw_IMU_stopped")
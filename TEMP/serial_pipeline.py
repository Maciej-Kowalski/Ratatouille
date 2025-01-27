import queue
import threading
import signal
import sys
import time
import serial
import struct
#import pdb

START_BYTE = 128
IMU_PACKET_LENGTH = 25
AUDIO_PACKET_LENGTH = 5

class CursorMouse:
    def __init__(self, serial_settings, filter, output):

        self.ser = serial.Serial(**serial_settings)

        self.filter = filter
        self.output = output

        # Queues for data passing between threads
        self.raw_data_queue = queue.Queue()
        self.processed_data_queue = queue.Queue()

        self.stop_event = threading.Event()
        # Threads
        self.threads = []
        self.running = False

    def start(self):
        """Start the pipeline by initializing threads."""
        self.running = True

        # Start the data streaming thread
        stream_thread = threading.Thread(target=self.start_stream, args=(self.raw_data_queue, self.stop_event))
        self.threads.append(stream_thread)

        process_thread = threading.Thread(target=self.filter, args=(self.raw_data_queue, self.processed_data_queue, self.stop_event))
        self.threads.append(process_thread)

        # Start output threads
        output_thread = threading.Thread(target=self.output, args=(self.processed_data_queue, self.stop_event))
        self.threads.append(output_thread)

        # Start all threads
        for thread in self.threads:
            thread.start()

        # Wait for KeyboardInterrupt to gracefully exit
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            self.stop_event.set()
            self.shutdown()
    
    def start_stream(self, input_queue, stop_event):
        while not stop_event.is_set():
            data = self.ser.read(IMU_PACKET_LENGTH)
            if data and data[0] == START_BYTE:  # Check for start byte (128)
                    self.unpack(data[1:IMU_PACKET_LENGTH])
            elif data:
                print("First byte - ", data[0])
            else:
                print("No data")
        print("start_stream_stopped")

    def unpack(self, data):
        if len(data) == IMU_PACKET_LENGTH-1:
            g_f32 = struct.unpack('fff', data[0:12])
            a_f32 = struct.unpack('fff', data[12:24])
            self.raw_data_queue.put((g_f32, a_f32))
        else:
            print("Data packet length invalid - ", len(data), ", expected ", IMU_PACKET_LENGTH-1)
        
    def shutdown(self):
        """Gracefully stop all threads."""
        print("\nShutting down...")
        self.running = False
        for thread in self.threads:
            thread.join()
        #sys.exit(0)


# Example of thread functions that take stop_event as an argument
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

def print_raw_IMU(input_queue, stop_event):
    while not stop_event.is_set():
        try:
            gyro, acc = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f"gyro: {[f'{x:8.4f}' for x in gyro]}, acc: {[f'{x:8.4f}' for x in acc]}")
            input_queue.task_done()
        except queue.Empty:
            continue
    print("raw_IMU_stopped")

class ClickMouse:
    def __init__(self, serial_settings, filter, output):

        self.ser = serial.Serial(**serial_settings)

        self.filter = filter
        self.output = output

        # Queues for data passing between threads
        self.raw_data_queue = queue.Queue()
        self.processed_data_queue = queue.Queue()

        self.stop_event = threading.Event()
        # Threads
        self.threads = []
        self.running = False

    def start(self):
        """Start the pipeline by initializing threads."""
        self.running = True

        # Start the data streaming thread
        stream_thread = threading.Thread(target=self.start_stream, args=(self.raw_data_queue, self.stop_event))
        self.threads.append(stream_thread)

        process_thread = threading.Thread(target=self.filter, args=(self.raw_data_queue, self.processed_data_queue, self.stop_event))
        self.threads.append(process_thread)

        # Start output threads
        output_thread = threading.Thread(target=self.output, args=(self.processed_data_queue, self.stop_event))
        self.threads.append(output_thread)

        # Start all threads
        for thread in self.threads:
            thread.start()

        # Wait for KeyboardInterrupt to gracefully exit
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.running = False
            self.stop_event.set()
            self.shutdown()
    
    def start_stream(self, input_queue, stop_event):
        while not stop_event.is_set():
            data = self.ser.read(AUDIO_PACKET_LENGTH)
            if data and data[0] == START_BYTE:  # Check for start byte (128)
                    self.unpack(data[1:AUDIO_PACKET_LENGTH])
            elif data:
                print("First byte - ", data[0])
            else:
                print("No data")
        print("start_stream_stopped")

    def unpack(self, data):
        if len(data) == AUDIO_PACKET_LENGTH-1:
            audio1, audio2 = struct.unpack('hh', data[0:4])
            self.raw_data_queue.put((audio1, audio2))
        else:
            print("Data packet length invalid - ", len(data), ", expected ", AUDIO_PACKET_LENGTH-1)
        
    def shutdown(self):
        """Gracefully stop all threads."""
        print("\nShutting down...")
        self.running = False
        for thread in self.threads:
            thread.join()
        #sys.exit(0)

def print_raw_audio(input_queue, stop_event):
    while not stop_event.is_set():
        try:
            audio1, audio2 = input_queue.get(timeout = 0.01)  # Use timeout to prevent hanging
            print(f"audio1: {audio1:8.4f}, acc: {[f'{x:8.4f}' for x in acc]}")
            input_queue.task_done()
        except queue.Empty:
            continue
    print("raw_IMU_stopped")

# Example usage
print("test")

def process():
    try:
        serial_settings = {
            "port": "COM16",
            "baudrate": 115200,
            "timeout": 1
        }
        prototype = CursorMouse(serial_settings, filter=no_filter, output=print_raw_IMU)
        prototype.start()
    except Exception as e:
        print(e)

process()

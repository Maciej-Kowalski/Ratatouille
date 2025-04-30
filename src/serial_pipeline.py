import queue
import threading
import signal
import sys
import time
import serial
import struct
import numpy as np
#import pdb

START_BYTE = 128

IMU_PACKET_LENGTH = 25

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

AUDIO_PACKET_LENGTH = 3

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
            audio1, audio2 = struct.unpack('hh', data[0:AUDIO_PACKET_LENGTH-1])
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

BUFFER_SIZE = 100
AUDIO_BUFFERED_PACKET_LENGTH = 2*BUFFER_SIZE + 1

class ClickMouseBuffered:
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
        output_thread = threading.Thread(target=self.output, args=(self.processed_data_queue, self.stop_event, BUFFER_SIZE ))
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
            data = self.ser.read(AUDIO_BUFFERED_PACKET_LENGTH)
            if data and data[0] == START_BYTE:  # Check for start byte (128)
                    self.unpack(data[1:AUDIO_BUFFERED_PACKET_LENGTH])
            #elif data:
                #print("First byte - ", data[0])
            #else:
                #print("No data")
        print("start_stream_stopped")

    def unpack(self, data):
        if len(data) == AUDIO_BUFFERED_PACKET_LENGTH-1:
            audio = np.frombuffer(data, dtype=np.int16)
            
            '''FOR DOUBLE MIC
            # Separate alternating values into audio1 and audio2
            audio1 = raw_values[0::2]  # Take even-indexed values
            audio2 = raw_values[1::2]  # Take odd-indexed values

            # Create a structured NumPy array for audio1 and audio2
            structured_audio = np.array(list(zip(audio1, audio2)), dtype=[('audio1', np.int16), ('audio2', np.int16)]) '''

            # Push the structured data to the queue
            self.raw_data_queue.put(audio)
        else:
            print("Data packet length invalid - ", len(data), ", expected ", AUDIO_BUFFERED_PACKET_LENGTH-1)
            print(data)
        
    def shutdown(self):
        """Gracefully stop all threads."""
        print("\nShutting down...")
        self.running = False
        for thread in self.threads:
            thread.join()
        #sys.exit(0)

FUSION_BUFFER_SIZE = 100
AUDIO_OFFSET = FUSION_BUFFER_SIZE * 2
ACCEL_OFFSET = AUDIO_OFFSET + 3 * 4
GYRO_OFFSET = ACCEL_OFFSET + 3 * 4
FUSION_PACKET_LENGTH = 1 + GYRO_OFFSET

class FusionMouse:
    def __init__(self, serial_settings, audio_filter, audio_output, IMU_filter, IMU_output):

        self.ser = serial.Serial(**serial_settings)

        self.audio_filter = audio_filter
        self.audio_output = audio_output

        self.IMU_filter = IMU_filter
        self.IMU_output = IMU_output

        # Queues for data passing between threads
        self.audio_raw_data_queue = queue.Queue()
        self.audio_processed_data_queue = queue.Queue()

        self.IMU_raw_data_queue = queue.Queue()
        self.IMU_processed_data_queue = queue.Queue()

        self.stop_event = threading.Event()
        # Threads
        self.threads = []
        self.running = False

    def start(self):
        """Start the pipeline by initializing threads."""
        self.running = True

        # Start the data streaming thread
        stream_thread = threading.Thread(target=self.start_stream)
        self.threads.append(stream_thread)

        audio_process_thread = threading.Thread(target=self.audio_filter, args=(self.audio_raw_data_queue, self.audio_processed_data_queue, self.stop_event))
        self.threads.append(audio_process_thread)

        # Start output threads
        audio_output_thread = threading.Thread(target=self.audio_output, args=(self.audio_processed_data_queue, self.stop_event, FUSION_BUFFER_SIZE))
        self.threads.append(audio_output_thread)

        IMU_process_thread = threading.Thread(target=self.IMU_filter, args=(self.IMU_raw_data_queue, self.IMU_processed_data_queue, self.stop_event))
        self.threads.append(IMU_process_thread)

        # Start output threads
        IMU_output_thread = threading.Thread(target=self.IMU_output, args=(self.IMU_processed_data_queue, self.stop_event))
        self.threads.append(IMU_output_thread)

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
    
    def start_stream(self):
        while not self.stop_event.is_set():
            data = self.ser.read(FUSION_PACKET_LENGTH)
            if data and data[0] == START_BYTE:  # Check for start byte (128)
                    self.unpack(data[1:FUSION_PACKET_LENGTH])
            elif data:
                print("First byte - ", data[0])
            else:
                print("No data")
        print("start_stream_stopped")

    def unpack(self, data):
        if len(data) == FUSION_PACKET_LENGTH-1:
            audio = np.frombuffer(data[0:AUDIO_OFFSET], dtype=np.int16)
            self.audio_raw_data_queue.put(audio)
            acc = struct.unpack('<fff', data[AUDIO_OFFSET:ACCEL_OFFSET])
            gyro = struct.unpack('<fff', data[ACCEL_OFFSET:GYRO_OFFSET])
            self.IMU_raw_data_queue.put((gyro,acc))
        else:
            print("Data packet length invalid - ", len(data), ", expected ", FUSION_PACKET_LENGTH-1)
            print(data)
        
    def shutdown(self):
        """Gracefully stop all threads."""
        print("\nShutting down...")
        self.running = False
        for thread in self.threads:
            thread.join()
        #sys.exit(0)

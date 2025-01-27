import queue
import threading
import signal
import sys
import time
import serial

class CursorMouse:
    def __init__(self, serial_settings, filter, output):
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
        # stream_thread = threading.Thread(target=self.data_source, daemon=True)
        # self.threads.append(stream_thread)

        process_thread = threading.Thread(target=self.filter, args=(self.stop_event))
        self.threads.append(process_thread)

        # Start output threads
        output_thread = threading.Thread(target=self.output, args=(self.stop_event))
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

    def shutdown(self):
        """Gracefully stop all threads."""
        print("\nShutting down...")
        self.running = False
        sys.exit(0)


# Example of thread functions that take stop_event as an argument
def printProcessing(stop_event):
    while not stop_event.is_set():
        print("Processing thread")
        time.sleep(1)

def printOutput(stop_event):
    while not stop_event.is_set():
        print("Output thread")
        time.sleep(1)

# Example usage
print("test")

serial_settings = {
    "port": "COM11",
    "baudrate": 115200,
    "timeout": 1
}
prototype = CursorMouse(serial_settings, filter=printProcessing, output=printOutput)
prototype.start()

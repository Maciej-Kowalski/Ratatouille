import queue
from threading import Thread
import signal
import sys
import time


class Pipeline:
    def __init__(self, data_source, mode, processing_funcs, output_funcs):
        """
        Initialize the pipeline with the required parameters.
        
        Args:
            data_source: Function to start data streaming (e.g., startstream).
            mode: The type of data (e.g., 'imu', 'audio', or 'both').
            processing_funcs: Dictionary of processing functions for each data type, e.g., {'imu': process_imu, 'audio': process_audio}.
            output_funcs: Dictionary of output functions for each data type, e.g., {'imu': output_imu, 'audio': output_audio}.
        """
        self.data_source = data_source
        self.mode = mode
        self.processing_funcs = processing_funcs
        self.output_funcs = output_funcs

        # Queues for data passing between threads
        self.raw_data_queue = queue.Queue()
        self.processed_data_queues = {key: queue.Queue() for key in processing_funcs.keys()}

        # Threads
        self.threads = []
        self.running = False

    def start(self):
        """Start the pipeline by initializing threads."""
        self.running = True

        # Start the data streaming thread
        stream_thread = Thread(target=self.data_source_thread, daemon=True)
        self.threads.append(stream_thread)

        # Start processing threads
        for data_type, process_func in self.processing_funcs.items():
            process_thread = Thread(target=self.processing_thread, args=(data_type, process_func), daemon=True)
            self.threads.append(process_thread)

        # Start output threads
        for data_type, output_func in self.output_funcs.items():
            output_thread = Thread(target=self.output_thread, args=(data_type, output_func), daemon=True)
            self.threads.append(output_thread)

        # Start all threads
        for thread in self.threads:
            thread.start()

        # Wait for KeyboardInterrupt to gracefully exit
        try:
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            self.shutdown()

    def data_source_thread(self):
        """Thread to fetch data and put it into the raw data queue."""
        while self.running:
            data = self.data_source()  # Fetch data from the source
            if data:
                self.raw_data_queue.put(data)

    def processing_thread(self, data_type, process_func):
        """Thread to process data and put it into the respective processed data queue."""
        while self.running:
            try:
                raw_data = self.raw_data_queue.get(timeout=1)
                processed_data = process_func(raw_data)  # Process the data
                self.processed_data_queues[data_type].put(processed_data)
                self.raw_data_queue.task_done()
            except queue.Empty:
                continue

    def output_thread(self, data_type, output_func):
        """Thread to handle the output of processed data."""
        while self.running:
            try:
                processed_data = self.processed_data_queues[data_type].get(timeout=1)
                output_func(processed_data)  # Perform the output action
                self.processed_data_queues[data_type].task_done()
            except queue.Empty:
                continue

    def shutdown(self):
        """Gracefully stop all threads."""
        print("\nShutting down pipeline...")
        self.running = False
        sys.exit(0)
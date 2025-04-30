from serial_pipeline import CursorMouse
from serial_pipeline import ClickMouse
from serial_pipeline import ClickMouseBuffered
from serial_pipeline import FusionMouse
from filters import no_filter
from filters import complementary_filter
from filters import EK_filter
from filters import audio_bandpass_high_F_filter
from outputs import print_counter_audio
from outputs import print_counter_audio_buffered
from outputs import timing_audio_buffered
from outputs import print_raw_IMU
from outputs import print_comp_IMU
from outputs import print_EK_filter_IMU
from outputs import mouse_cursor_mapping
from outputs import mouse_cursor_mapping_no_cal
from outputs import plot_audio_buffered
from outputs import no_output
from outputs import print_continuous_audio
from outputs import plot_continuous_audio
from outputs import plot_audio_with_fft
from outputs import record_and_plot_audio

def audioBuff():
    try:
        serial_settings = {
            "port": "COM9",
            "baudrate": 115200,
            "timeout": 1
        }
        prototype = ClickMouseBuffered(serial_settings, filter=no_filter, output=print_continuous_audio)
        #prototype = ClickMouseBuffered(serial_settings, filter=audio_bandpass_high_F_filter, output=record_and_plot_audio)
        prototype.start()
    except Exception as e:
        print(e)

def audioBufftiming():
    try:
        serial_settings = {
            "port": "COM19",
            "baudrate": 115200,
            "timeout": 1
        }
        prototype = ClickMouseBuffered(serial_settings, filter=no_filter, output=timing_audio_buffered)
        #prototype = ClickMouseBuffered(serial_settings, filter=audio_bandpass_high_F_filter, output=record_and_plot_audio)
        prototype.start()
    except Exception as e:
        print(e)

def audio():
    try:
        serial_settings = {
            "port": "COM19",
            "baudrate": 115200,
            "timeout": 1
        }
        prototype = ClickMouse(serial_settings, filter=no_filter, output=print_counter_audio)
        prototype.start()
    except Exception as e:
        print(e)

def IMU():
    try:
        serial_settings = {
            "port": "COM16",
            "baudrate": 115200,
            "timeout": 1
        }
        prototype = CursorMouse(serial_settings, filter=EK_filter, output=mouse_cursor_mapping)
        #prototype = CursorMouse(serial_settings, filter=EK_filter, output=print_EK_filter_IMU)
        prototype.start()
    except Exception as e:
        print(e)

def Fusion():
    try:
        serial_settings = {
            "port": "COM16",
            "baudrate": 115200,
            "timeout": 1
        }
        prototype = FusionMouse(serial_settings, audio_filter=no_filter, audio_output=print_counter_audio_buffered, IMU_filter=no_filter, IMU_output=print_raw_IMU)
        #prototype = CursorMouse(serial_settings, filter=EK_filter, output=print_EK_filter_IMU)
        prototype.start()
    except Exception as e:
        print(e)

print("Start")
Fusion()
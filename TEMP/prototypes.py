from serial_pipeline import CursorMouse
from serial_pipeline import ClickMouse
from serial_pipeline import ClickMouseBuffered
from filters import no_filter
from filters import complementary_filter
from filters import EK_filter
from outputs import print_counter_audio
from outputs import print_counter_audio_buffered
from outputs import print_raw_IMU
from outputs import print_comp_IMU
from outputs import print_EK_filter_IMU
from outputs import mouse_cursor_mapping
from outputs import mouse_cursor_mapping_no_cal

def audioBuff():
    try:
        serial_settings = {
            "port": "COM16",
            "baudrate": 115200,
            "timeout": 1
        }
        prototype = ClickMouseBuffered(serial_settings, filter=no_filter, output=print_counter_audio_buffered)
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

print("test")
audioBuff()
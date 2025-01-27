#from serial_pipeline import CursorMouse
from serial_pipeline import ClickMouse
from filers import no_filter
from outputs import print_counter_audio

def process():
    try:
        serial_settings = {
            "port": "COM16",
            "baudrate": 115200,
            "timeout": 1
        }
        prototype = ClickMouse(serial_settings, filter=no_filter, output=print_counter_audio)
        prototype.start()
    except Exception as e:
        print(e)

print("test")
process()
import serial

# Replace 'COM16' with your actual COM port
COM_PORT = "COM11"
BAUD_RATE = 115200  # Match this to your STM32's UART configuration

try:
    # Open the serial port
    with serial.Serial(COM_PORT, BAUD_RATE, timeout=1) as ser:
        print(f"Listening on {COM_PORT} at {BAUD_RATE} baud...")
        
        while True:
            # Read incoming data
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)  # Read all available data
                print(f"{repr(data.decode('utf-8'))}")
except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("\nExiting...")

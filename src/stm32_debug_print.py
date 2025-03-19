import serial

# Replace with your STM32's COM port and baud rate
com_port = 'COM11'  # e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux
baud_rate = 115200

# Open the serial port
with serial.Serial(com_port, baud_rate, timeout=1) as ser:
    while True:
        # Read data from the serial port
        if ser.in_waiting > 0:
            message = ser.readline().decode('utf-8').strip()
            print(message)
        else:
            print("1")
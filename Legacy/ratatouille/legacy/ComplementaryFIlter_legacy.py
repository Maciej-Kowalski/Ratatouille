import math
import time

# Constants and initial values
Pi = math.pi
alpha = 0.97  # Filter coefficient
dt = 0.01  # Time step

gyroscale = 1 / 262.4  # Gyroscope scale calculated from LSB (datasheet)

# Initial estimated angles
pitch_est = 0
roll_est = 0

# Main loop
while True:
    # READ raw data from IMU (placeholder constants)
    ax = 0.01
    ay = 0.02
    az = 0.03
    gx = 0.04
    gy = 0.05
    gz = 0.06

    # Conversion of acceleration values (LSB from datasheet)
    ax = ax / 16384
    ay = ay / 16384
    az = az / 16384

    # Gyroscope scaling
    pitch_g = gx * gyroscale * Pi / 180  # Converting to degrees per second
    roll_g = gy * gyroscale * Pi / 180

    # Accelerometer angle estimation
    pitch_a = math.atan2(ay, math.sqrt(ax**2 + az**2)) * 180 / Pi
    roll_a = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / Pi

    # Combination of estimates with weight alpha
    pitch_est = alpha * (pitch_est + pitch_g * dt) + (1 - alpha) * pitch_a
    roll_est = alpha * (roll_est + roll_g * dt) + (1 - alpha) * roll_a

    print(f"Filtered Angles: Pitch = {pitch_est:.2f}, Roll = {roll_est:.2f}")

    # Simulate time step (for real application, use actual timing)
    time.sleep(dt)
#!/usr/bin/env python3
# Lab 11 Task 1: live plot of the tilt angle
# Board sends one line every 50 ms:  acc_angle,gyro_angle,filtered_angle  (degrees)
# Set PLOT_MODE 1 in main.c so the board sends plain CSV.

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

PORT = "/dev/ttyACM0"     # ST-LINK V2-1 virtual COM port (USART1)
BAUD = 115200
WINDOW = 200              # samples on screen: 200 x 50 ms = 10 s
SAMPLE_S = 0.05           # 100 Hz loop, print every 5th loop -> 20 lines/s

ser = serial.Serial(PORT, BAUD, timeout=0.1)

acc_data = deque(maxlen=WINDOW)
gyro_data = deque(maxlen=WINDOW)
filt_data = deque(maxlen=WINDOW)
time_data = deque(maxlen=WINDOW)
count = 0

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)


def update(_frame):
    global count

    # Read every line that has arrived since the last frame
    while ser.in_waiting:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            parts = line.split(",")
            if len(parts) == 3:              # skip start-up text and half lines
                acc_data.append(float(parts[0]))
                gyro_data.append(float(parts[1]))
                filt_data.append(float(parts[2]))
                time_data.append(count * SAMPLE_S)
                count += 1
        except ValueError:
            pass

    t = list(time_data)

    # Top: accelerometer vs filtered -> shows the filter removing noise
    ax1.clear()
    ax1.plot(t, list(acc_data), label="Accel angle (X, Z)", color="tab:orange", alpha=0.7)
    ax1.plot(t, list(filt_data), label="Filtered angle", color="tab:green", linewidth=2)
    ax1.set_title("Tilt angle: complementary filter")
    ax1.set_ylabel("Angle (deg)")
    ax1.set_ylim(-90, 90)
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    # Bottom: gyro vs filtered -> shows the filter removing drift
    ax2.clear()
    ax2.plot(t, list(gyro_data), label="Gyro angle (Y)", color="tab:blue")
    ax2.plot(t, list(filt_data), label="Filtered angle", color="tab:green", linewidth=2)
    ax2.set_ylabel("Angle (deg)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylim(-90, 90)
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)


ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.tight_layout()
plt.show()
ser.close()

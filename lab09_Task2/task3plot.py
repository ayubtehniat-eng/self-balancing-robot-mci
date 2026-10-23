import serial
import matplotlib.pyplot as plt
from drawnow import *

# === Setup Serial ===
sinWaveData = serial.Serial('/dev/ttyUSB0', 115200)
plt.ion()

# === Plot Styles to Rotate Through ===
plot_styles = ['r.-', 'b.-', 'g.-', 'm.-', 'c.-', 'y.-', 'k.-']

# === Data Storage ===
NUM_SAMPLES = 100
time_ms = []
cnt = 0
all_signals = []  # all_signals[0] = temp, [1] = X, [2] = Y, [3] = Z
signals = ["Temperature", "X", "Y", "Z"]


# === Plotting Function ===
def makeFig(title, xLabel, yLabel, yLimit, *args):
    plt.clf()
    plt.title(title)
    plt.grid(True)
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    plt.ylim(yLimit[0], yLimit[1])

    for arg in args:
        plt.plot(time_ms, arg[0], arg[1], label=arg[2])

    plt.legend(loc='upper left')


# === Main Loop ===
while True:
    while sinWaveData.inWaiting() == 0:
        pass  # Wait for data

    try:
        line = sinWaveData.readline().decode(errors='ignore').strip()
        values = line.split(',')

        # CHANGED: skip incomplete lines (e.g. the first, cut-off line)
        if len(values) != len(signals):
            continue

        # First time: initialize sublists
        if len(all_signals) != len(values):
            all_signals = [[] for _ in range(len(values))]

        # CHANGED: float() instead of int(), since X/Y/Z are decimals like 0.23
        for i, val in enumerate(values):
            all_signals[i].append(float(val))

        time_ms.append(cnt)
        cnt += 1

        # Build args for plotting
        args = []
        for i, signal in enumerate(all_signals):
            style = plot_styles[i % len(plot_styles)]
            label = signals[i]
            args.append((signal, style, label))

        title = "Gyroscope Plot"
        drawnow(lambda: makeFig(title, "Time (ms)", "Value", [-255, 255], *args))
        plt.pause(0.0001)

        # Keep only last NUM_SAMPLES samples
        if len(time_ms) > NUM_SAMPLES:
            time_ms.pop(0)
            for signal in all_signals:
                signal.pop(0)

    except Exception as e:
        print("Error:", e)
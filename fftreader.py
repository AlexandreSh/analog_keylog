import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
import pandas as pd
from datetime import datetime
import signal
import sys

def find_arduino():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        print(f"Checking port {port.device} with hwid {port.hwid}")
        if '1A86' in port.hwid and '7523' in port.hwid:
            print(f"Arduino found on port {port.device}")
            return port.device
    return None

def save_data_with_timestamp(sensor_data, filename='sensor_data.txt'):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S').ljust(40)
    with open(filename, 'a') as file:
        file.write(f"----- begin of {timestamp} -----\n")
        for line in sensor_data:
            file.write(line + '\n')
        file.write(f"----- end of {timestamp} -----\n")

def read_data(ser, timeout=30):
    sensor_data = [[] for _ in range(8)]
    batch_data = []
    start_time = time.time()

    while True:
        if time.time() - start_time > timeout:
            print("No data received for 30 seconds. Shutting down.")
            save_data_with_timestamp(batch_data)  # Save any collected data
            return sensor_data  # Return the collected data even if it's incomplete

        try:
            line = ser.readline().decode('utf-8').strip()
            print(f"Read line: {line}")  # Debug print
            batch_data.append(line)  # Accumulate the received line
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            save_data_with_timestamp(batch_data)  # Save any collected data
            break

        if line:
            start_time = time.time()  # Reset the timer on receiving data

        if line == "---":
            save_data_with_timestamp(batch_data)  # Save the completed batch
            batch_data = []  # Clear the batch data for the next batch
            break

        parts = line.split(" Hz: ")
        if len(parts) < 2:
            print(f"Skipping malformed line: {line}")  # Debug print
            continue

        try:
            freq = float(parts[0])
            sensor_readings = parts[1].split(" Sensor ")
            for sensor_reading in sensor_readings[1:]:
                sensor_info = sensor_reading.split(": ")
                if len(sensor_info) == 2:
                    sensor = int(sensor_info[0])
                    mag = float(sensor_info[1])
                    if sensor < len(sensor_data):  # Ensure sensor index is in range
                        sensor_data[sensor].append((freq, mag))
        except ValueError:
            print(f"Skipping malformed line: {line}")  # Debug print

    return sensor_data

# Variables to store highest and average magnitudes for each frequency and sensor
max_magnitudes = {}
avg_magnitudes = {}
frequency_counts = {}

def update_statistics(sensor_data):
    global max_magnitudes, avg_magnitudes, frequency_counts
    for sensor_id, sensor in enumerate(sensor_data):
        for freq, mag in sensor:
            if freq not in max_magnitudes:
                max_magnitudes[freq] = {}
                avg_magnitudes[freq] = {}
                frequency_counts[freq] = {}
            if sensor_id not in max_magnitudes[freq]:
                max_magnitudes[freq][sensor_id] = mag
                avg_magnitudes[freq][sensor_id] = mag
                frequency_counts[freq][sensor_id] = 1
            else:
                max_magnitudes[freq][sensor_id] = max(max_magnitudes[freq][sensor_id], mag)
                avg_magnitudes[freq][sensor_id] = (avg_magnitudes[freq][sensor_id] * frequency_counts[freq][sensor_id] + mag) / (frequency_counts[freq][sensor_id] + 1)
                frequency_counts[freq][sensor_id] += 1

fig, (ax1, ax2) = plt.subplots(2, 1)
def plot_data(sensor_data):
    global fig, ax1, ax2
    bars = []
    width = 0.1

    def init():
        for sensor in range(len(sensor_data)):
            bar = ax1.bar([0], [0], width=width, label=f'Sensor {sensor}')
            bars.append(bar)
        ax1.set_xlabel('Frequency (Hz)')
        ax1.set_ylabel('Magnitude (log scale)')
        ax1.set_title('FFT Data from Sensors')
        ax1.legend()

        ax2.set_xlabel('Frequency (Hz)')
        ax2.set_ylabel('Magnitude (log scale)')
        ax2.set_title('Highest Detections')
        ax2.set_yscale('log')

        return bars

    def update(frame):
        sensor_data = read_data(ser, timeout=2)
        update_statistics(sensor_data)
        ax1.clear()
        ax2.clear()
        max_mag = 0
        has_positive_values = False
        for sensor, data in enumerate(sensor_data):
            if data:
                freqs, mags = zip(*data)
                ax1.bar([f + sensor * width for f in freqs], mags, width=width, label=f'Sensor {sensor}')
                max_mag = max(max_mag, max(mags))
                if max(mags) > 0:
                    has_positive_values = True
        ax1.set_xlabel('Frequency (Hz)')
        ax1.set_ylabel('Magnitude (log scale)')
        ax1.set_title('FFT Data from Sensors')
        ax1.legend()
        if has_positive_values:
            ax1.set_yscale('log')
        else:
            ax1.set_yscale('linear')
        ax1.set_ylim(1, max_mag * 1.1)  # Adjust the y-axis to slightly above the max magnitude

        if max_magnitudes:
            freqs = sorted(max_magnitudes.keys())
            max_mags = [max(max_magnitudes[freq].values()) for freq in freqs]
            ax2.bar(freqs, max_mags, width=width)
            ax2.set_xlabel('Frequency (Hz)')
            ax2.set_ylabel('Magnitude (log scale)')
            ax2.set_title('Highest Detections')
            if any(m > 0 for m in max_mags):
                ax2.set_yscale('log')
            else:
                ax2.set_yscale('linear')
            ax2.set_ylim(1, max(max_mags) * 1.1)  # Adjust the y-axis to slightly above the max magnitude

        display_table()

        return bars

    ani = animation.FuncAnimation(fig, update, init_func=init, blit=False, interval=2000, cache_frame_data=False)
    plt.show()

def display_table():
    if max_magnitudes:
        print(f"\n{'Frequency (Hz)':<15}{'Sensor Data':<50}")
        for freq in sorted(max_magnitudes.keys()):
            sensor_data = []
            for sensor_id in sorted(max_magnitudes[freq].keys()):
                sensor_data.append(f"Sensor {sensor_id}: {max_magnitudes[freq][sensor_id]:.2f} : {avg_magnitudes[freq][sensor_id]:.2f}")
            print(f"{freq:<15}{'; '.join(sensor_data):<50}")

def save_figure():
    timestamp = datetime.now().strftime('%Y-%m-%d %H-%M-%S').ljust(40)
    fig.savefig(f"FFT_{timestamp}.png")

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Saving the last figure...')
    save_figure()
    if ser.is_open:
        ser.close()
    print("Serial port closed.")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Main loop
arduino_port = find_arduino()
if arduino_port is None:
    print("Arduino not found.")
else:
    try:
        ser = serial.Serial(arduino_port, 115200)
        print(f"Connected to Arduino on port {arduino_port}")
        time.sleep(2)  # Wait for Arduino to reset

        plot_data([[] for _ in range(8)])
    except serial.SerialException as e:
        print(f"Failed to connect to Arduino: {e}")
    finally:
        if ser.is_open:
            ser.close()
        print("Serial port closed.")
        save_figure()

import serial
import json
from json.decoder import JSONDecodeError
import csv
import os
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

PORT = input("enter the port (e.g. 'COM3' for windows)")
BAUDRATE = 9600
MAX_POINTS = 100
CSV_FILENAME = f"pico_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# ==== Serial setup ====
ser = serial.Serial(PORT, BAUDRATE, timeout=1)

# ==== Data buffers ====
x_data = deque(maxlen=MAX_POINTS)
data_buffers = {}
counter = 0

# ==== Plot setup ====
fig, ax = plt.subplots()
lines = {}
colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange', 'tab:purple']

# ==== CSV setup ====
csv_file = open(CSV_FILENAME, mode='w', newline='')
csv_writer = None
csv_header_written = False


def write_to_csv(data_dict):
    global csv_writer, csv_header_written

    if not csv_header_written:
        fieldnames = ['timestamp'] + list(data_dict.keys())
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()
        csv_header_written = True

    row = {'timestamp': datetime.now().isoformat(), **data_dict}
    if csv_writer is not None:
        csv_writer.writerow(row)
    csv_file.flush()  # Ensure it's written immediately


def update(frame):
    global counter

    # Read serial data
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8').strip()
            data = json.loads(line)

            # Save to CSV
            write_to_csv(data)

            # Plotting logic
            x_data.append(counter)
            counter += 1

            for i, (key, value) in enumerate(data.items()):
                if key not in data_buffers:
                    data_buffers[key] = deque(maxlen=MAX_POINTS)
                    lines[key], = ax.plot([], [], label=key, color=colors[i % len(colors)])
                data_buffers[key].append(value)
        except JSONDecodeError:
            print("Invalid JSON received.")
        except Exception as e:
            print("Error:", e)

    # Update plot
    for key, line_obj in lines.items():
        y_vals = list(data_buffers[key])
        x_vals = list(x_data)[-len(y_vals):]
        line_obj.set_data(x_vals, y_vals)

    if x_data:
        ax.set_xlim(max(0, counter - MAX_POINTS), counter)
        all_y = [val for buffer in data_buffers.values() for val in buffer]
        if all_y:
            y_min = min(all_y)
            y_max = max(all_y)
            ax.set_ylim(y_min - 5, y_max + 5)

    ax.set_title("Live JSON Data from Pico")
    ax.set_xlabel("Samples")
    ax.set_ylabel("Values")
    ax.legend(loc="upper left")
    return lines.values()


ani = animation.FuncAnimation(fig, update, blit=False, interval=200)
plt.tight_layout()
plt.grid(True)

try:
    plt.show()
finally:
    print("Closing CSV and serial connection.")
    csv_file.close()
    ser.close()

import serial
import csv
import time

PORT = 'COM5'
BAUD = 115200
OUTPUT_FILE = 'Night 1.csv'

ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Logging to {OUTPUT_FILE}... Ctrl+C to stop")

with open(OUTPUT_FILE, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'aX', 'aY', 'aZ'])
    
    while True:
        line = ser.readline().decode('utf-8').strip()
        if ',' in line and line[0].isdigit():
            try:
                parts = line.split(',')
                timestamp, ax, ay, az = parts[0], parts[1], parts[2], parts[3]
                writer.writerow([timestamp, ax, ay, az])
                f.flush()
                print(f"{timestamp}, {ax}, {ay}, {az}")
            except:
                pass
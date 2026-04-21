import serial
import csv

ser = serial.Serial('COM3', 115200, timeout=1)

f = open("imu_data.csv", "w", newline="")
writer = csv.writer(f)

writer.writerow(["t", "qw", "qx", "qy", "qz"])

print("Logging...")

while True:
    line = ser.readline().decode(errors='ignore').strip()
    print(line)

    parts = line.split(",")

    if len(parts) == 5:
        try:
            t  = float(parts[0])
            qw = float(parts[1])
            qx = float(parts[2])
            qy = float(parts[3])
            qz = float(parts[4])

            writer.writerow([t, qw, qx, qy, qz])
            f.flush()

            print("WROTE:", t)

        except:
            pass
from vpython import *
import serial
import math

PORT = 'COM3'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

scene = canvas(title="IMU Gizmo", width=800, height=600, background=color.black)

# 🔹 Thin axis arrows (Blender style)
x_axis = arrow(pos=vector(0,0,0), axis=vector(2,0,0),
               color=color.red, shaftwidth=0.05)

y_axis = arrow(pos=vector(0,0,0), axis=vector(0,2,0),
               color=color.green, shaftwidth=0.05)

z_axis = arrow(pos=vector(0,0,0), axis=vector(0,0,2),
               color=color.blue, shaftwidth=0.05)

# 🔹 IMU body (small cube)
imu = box(pos=vector(0,0,0), size=vector(0.4,0.4,0.4), color=color.white)

# Quaternion → rotation matrix
def quat_to_matrix(qw, qx, qy, qz):
    return [
        [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]
    ]

print("Live IMU started...")

while True:
    rate(60)

    try:
        line = ser.readline().decode(errors='ignore').strip()
        data = line.split(",")

        if len(data) < 5:
            continue

        qw, qx, qy, qz = map(float, data[1:5])

        # normalize
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm

        R = quat_to_matrix(qw, qx, qy, qz)

        # rotate axes (this is the key difference 👇)
        x_axis.axis = vector(R[0][0], R[1][0], R[2][0]) * 2
        y_axis.axis = vector(R[0][1], R[1][1], R[2][1]) * 2
        z_axis.axis = vector(R[0][2], R[1][2], R[2][2]) * 2

        # rotate cube
        imu.axis = x_axis.axis
        imu.up   = z_axis.axis

    except:
        pass
import rtde_control #import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface
import requests
import time
import math


STM32_URL = "http://172.31.1.133/api/IMU_data"
ROBOT_IP = "172.31.1.144"

print("booted")
# connect
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
print("has control")
rtde_r = RTDEReceiveInterface(ROBOT_IP)
print("has read")

print("sucessfull")

    #base #shoulder #elbow   #wrist1  #wrist2  #wrist3
home = [3.07, -2.20, 1.96, 0.93, 1.57, 0.00]

# Safety margins
SAFETY_MARGIN = math.radians(20)
MAX_OFFSET = 2*math.pi - SAFETY_MARGIN
MIN_OFFSET = -2*math.pi + SAFETY_MARGIN

# Control parameters
ALPHA = 0.2        # smoothing factor
DEADZONE = math.radians(5)
SCALE = 0.001      # scaling IMU delta to radians

def smooth(new, prev, alpha=ALPHA):
    return alpha*new + (1-alpha)*prev

def get_imu_data():
    try:
        r = requests.get(STM32_URL, timeout=1)
        if r.status_code == 200:
            return r.json()
    except Exception as e:
        print(f"IMU read failed: {e}")
    return None

def imu_delta(new, prev):
    delta = (new - prev) * SCALE
    if abs(delta) < DEADZONE:
        return 0.0
    return delta

try:
    # Initial joints
    joints = rtde_r.getActualQ()
    
    # Previous IMU readings (initialize to first read)
    imu_prev = get_imu_data()
    if imu_prev is None:
        imu_prev = {'accel_x':0, 'accel_y':0, 'accel_z':0}

    while True:
        imu = get_imu_data()
        if imu is None:
            continue

        # Read current joint positions
        current_joints = rtde_r.getActualQ()

        # Compute deltas from previous IMU values
        dx = imu_delta(imu['accel_x'], imu_prev['accel_x'])
        dy = imu_delta(imu['accel_y'], imu_prev['accel_y'])
        dz = imu_delta(imu['accel_z'], imu_prev['accel_z'])

        # Smooth deltas
        dx_s = smooth(dx, 0.0)
        dy_s = smooth(dy, 0.0)
        dz_s = smooth(dz, 0.0)

        # Incrementally update wrist joints
        joints[3] = current_joints[3] + dx_s
        joints[4] = current_joints[4] + dy_s
        joints[5] = current_joints[5] + dz_s

        # Clamp to safe range
        joints[3] = max(min(joints[3], MAX_OFFSET), MIN_OFFSET)
        joints[4] = max(min(joints[4], MAX_OFFSET), MIN_OFFSET)
        joints[5] = max(min(joints[5], MAX_OFFSET), MIN_OFFSET)


        # --- Debug print ---
        print(f"IMU: {imu}, Deltas(rad): [{dx_s:.4f}, {dy_s:.4f}, {dz_s:.4f}], "
              f"Joints(rad): [{joints[3]:.4f}, {joints[4]:.4f}, {joints[5]:.4f}]")

        # Send command to robot
        rtde_c.servoJ(joints, 0.2, 0.2, 0.8, 0.2, 200)

        # Update previous IMU readings
        imu_prev = imu

        time.sleep(0.008)

except KeyboardInterrupt:
    print("Stopping wrist control")
    rtde_c.stopJ()
# UR5e IMU Mirroring

A prototype for sending accelerometer readings from an **MMA8451 IMU** to an **UR5e** robot, then using changes in those readings to command the robot’s wrist joints.

The project has two parts:

- **STM32 firmware** reads the IMU over I²C and serves its readings as JSON over Ethernet.
- **Python controller** polls that endpoint and sends incremental wrist-joint commands to the UR5e using RTDE.

> **Safety:** This is an experimental control project, not a safety-rated system. Test with the robot in a controlled environment, at reduced speed, and with an accessible emergency stop. Review the motion limits and control behavior before connecting to a robot.

## Hardware and software

### Firmware

- STM32F767ZITx board
- MMA8451 accelerometer connected over I²C1
- Ethernet connection
- STM32CubeIDE / STM32CubeMX project (`f767.ioc`)
- FreeRTOS and the Mongoose embedded networking library

### Robot controller

- Python 3
- UR5e reachable over the network
- Python packages: `ur-rtde` and `requests`

## How it works

1. The firmware initializes the MMA8451 and reads its X, Y, and Z acceleration values.
2. Mongoose serves the IMU data through an HTTP API.
3. `IMU_converter.py` polls the API, compares each reading with the previous one, applies a deadzone and scale factor, and commands the robot with `servoJ`.
4. The Python script maps the three accelerometer axes to wrist joints 4–6.

The current controller parameters—including the deadzone, scaling, and servo settings—are defined near the top of `IMU_converter.py`.

## Network configuration

The checked-in configuration contains these example addresses:

| Device | Address in the source |
| --- | --- |
| STM32 firmware | `172.31.1.133` |
| UR5e | `172.31.1.144` |
| Firmware gateway | `172.31.1.1` |
| Firmware subnet mask | `255.255.255.0` |

These are private-network addresses, not universal defaults. Update them to match your network:

- In `mongoose/mongoose_config.h`, configure the firmware IP, gateway, and subnet mask.
- In `IMU_converter.py`, update `STM32_URL` and `ROBOT_IP`.

The firmware’s IMU endpoint is:

```text
GET http://<STM32-IP>/api/IMU_data
```

It returns JSON fields for `accel_x`, `accel_y`, `accel_z`, and `timestamp`.

## Build and run

### 1. Build the STM32 firmware

1. Open the project in STM32CubeIDE, or import it as an existing STM32 project.
2. Check the board, I²C, Ethernet, and network configuration for your hardware.
3. Build and flash the firmware to the STM32F767 board.
4. Connect the board to the same network as the computer running the Python controller.
5. Confirm that the IMU API responds at `http://<STM32-IP>/api/IMU_data`.

The repository includes the CubeMX configuration in `f767.ioc`. The firmware currently configures the MMA8451 on I²C1.

### 2. Install Python dependencies

```bash
python -m pip install ur-rtde requests
```

### 3. Configure and run the robot controller

1. Set `STM32_URL` and `ROBOT_IP` in `IMU_converter.py`.
2. Verify the robot is in a safe state and that the configured motion behavior is appropriate for your setup.
3. Run the controller:

```bash
python IMU_converter.py
```

Press **Ctrl+C** to stop the script; it calls `stopJ()` when interrupted.

## Repository layout

- `Core/` — STM32 application and peripheral initialization
- `Drivers/` — STM32 device and HAL drivers
- `Middlewares/` — middleware, including FreeRTOS
- `mongoose/` — embedded HTTP/networking library and generated API glue
- `f767.ioc` — STM32CubeMX project configuration
- `IMU_converter.py` — Python script connecting IMU data to UR5e wrist commands

## Limitations

- The controller uses fixed network addresses and motion parameters that must be reviewed and configured for your setup.
- Accelerometer changes are used directly as motion inputs; this is not a calibrated orientation or force-control system.
- The script communicates with the robot continuously and does not replace the robot’s built-in safety functions.
- This repository does not currently document a validated wiring diagram, calibration procedure, or production-ready deployment process.

- [Video Guides](https://www.youtube.com/@mongoose-networking-library/videos)
- [Ethernet hardware](https://mongoose.ws/documentation/tutorials/hardware/)
- [Firmware OTA updates](https://mongoose.ws/documentation/tutorials/firmware-update/)
- [Mongoose User Guide](https://mongoose.ws/documentation/)

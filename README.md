# Sequoia ROS2 Wrapper

ROS2 packages for interfacing with the Parrot Sequoia multispectral camera.

## Packages

### ptpy_vendor
Vendor package for the [sequoia-ptpy](https://github.com/Parrot-Developers/sequoia-ptpy) Python library, which provides PTP (Picture Transfer Protocol) communication with the Sequoia camera.

### sequoia_camera
ROS2 component package that publishes sensor data from the Sequoia camera and provides services to control image capture.

## Features

- **IMU Data Publishing**: Publishes gyroscope, accelerometer, orientation and magnetometer data
- **Sunshine Sensor**: Publishes multispectral irradiance data (Green, Red, RedEdge, NIR)
- **Camera Control**: Service interface for single capture and timelapse modes
- **Power Saving**: Automatically disables GPS and WiFi to conserve power and reduce interference
- **Component Architecture**: Can be composed with other components in a single process

## Building

```bash
cd sequoia_ros2_wrapper
colcon build
source install/setup.bash
```

## Usage

### Launch the camera node

``NOTE:`` Sensor data publishing disabled by default

```
ros2 launch sequoia_camera sequoia_camera.launch.py
```

### Topics

- `~/imu` (sensor_msgs/Imu)
- `~/mag/data` (sensor_msgs/MagneticField)
- `~/sunshine` ([sequoia_camera/SunshineSensor](sequoia_camera/msg/SunshineSensor.msg))

### Services

- `~/camera_control` ([sequoia_camera/CameraControl](sequoia_camera/srv/CameraControl.srv))

#### Camera Control Service Examples

**Single Capture:**
```bash
ros2 service call /sequoia_camera/camera_control sequoia_camera/srv/CameraControl "{mode: 1}"
```

**Start Timelapse (1 second interval):**
```bash
ros2 service call /sequoia_camera/camera_control sequoia_camera/srv/CameraControl "{mode: 2, timelapse_interval: 10.0}"
```

**Stop Capture:**
```bash
ros2 service call /sequoia_camera/camera_control sequoia_camera/srv/CameraControl "{mode: 0}"
```

## Note

### Permission
Run the [script](sequoia_camera/scripts/unmount_sequoia.sh) before launching the node as by default the Camera will be mounted as storage device.

```
./sequoia_camera/scripts/unmount_sequoia.sh
```

### Wifi
Press the button on the camera 4 times quickly to re-enable WiFi.

### General
```
Timelapse at every 10s interval, as USB device will response as busy
```
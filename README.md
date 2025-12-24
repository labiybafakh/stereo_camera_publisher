# Stereo Camera Publisher

ROS2 package for publishing stereo images from the 3D Global Shutter Camera (ID 32e4:9282) using V4L2 interface.

## Features

- Publishes synchronized stereo image pairs from left and right cameras
- Configurable image resolution and frame rate
- Adjustable camera parameters (exposure, gain)
- Support for camera calibration files
- Consistent device naming through udev rules

## Hardware

- **Camera**: 3D Global Shutter Camera
- **USB ID**: 32e4:9282
- **Interface**: V4L2 (Video4Linux2)
- **Video Devices**: Creates two video devices (left and right cameras)

## Installation

### 1. Install Dependencies

The package requires V4L2 development libraries (already installed on most systems):

```bash
sudo apt-get install libv4l-dev v4l-utils
```

### 2. Setup udev Rules

**Option A: Using the installation script (recommended)**

Run the provided installation script:

```bash
cd ~/vio_ws/src/stereo_camera_publisher
./install_udev_rules.sh
```

The script will automatically install the udev rules and verify the symlinks were created.

**Option B: Manual installation**

Copy the udev rules file manually:

```bash
cd ~/vio_ws/src/stereo_camera_publisher
sudo cp 99-stereo-camera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Verify the symlinks were created:

```bash
ls -l /dev/stereo_*
```

You should see:
```
lrwxrwxrwx 1 root root 6 ... /dev/stereo_left -> video2
lrwxrwxrwx 1 root root 6 ... /dev/stereo_right -> video3
```

### 3. Build the Package

```bash
cd ~/vio_ws
colcon build --packages-select stereo_camera_publisher
source install/setup.bash
```

## Configuration

Edit `config/stereo_camera_params.yaml` to adjust camera settings:

```yaml
stereo_camera_publisher:
  ros__parameters:
    # Camera device paths
    left_device: "/dev/stereo_left"
    right_device: "/dev/stereo_right"

    # Image resolution
    image_width: 1280
    image_height: 720

    # Frame rate in Hz
    frame_rate: 30.0

    # ROS topic names
    left_topic: "/stereo/left/image_raw"
    right_topic: "/stereo/right/image_raw"

    # TF frame IDs
    left_frame_id: "stereo_left"
    right_frame_id: "stereo_right"

    # Camera parameters
    exposure: 100
    gain: 50

    # Camera calibration files (optional)
    left_camera_info_url: ""
    right_camera_info_url: ""
```

### Configurable Parameters

- **image_width**: Image width in pixels (default: 1280)
- **image_height**: Image height in pixels (default: 720)
- **frame_rate**: Publishing rate in Hz (default: 30.0)
- **left_topic/right_topic**: Output topic names
- **exposure**: Camera exposure value (camera-dependent)
- **gain**: Camera gain value (camera-dependent)

## Usage

### Launch with Default Configuration

```bash
ros2 launch stereo_camera_publisher stereo_camera.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch stereo_camera_publisher stereo_camera.launch.py \
    image_width:=640 \
    image_height:=480 \
    frame_rate:=60.0
```

### Run Node Directly

```bash
ros2 run stereo_camera_publisher stereo_camera_node
```

## Published Topics

- `/stereo/left/image_raw` (sensor_msgs/Image) - Left camera raw images
- `/stereo/right/image_raw` (sensor_msgs/Image) - Right camera raw images
- `/stereo/left/image_raw_info` (sensor_msgs/CameraInfo) - Left camera info
- `/stereo/right/image_raw_info` (sensor_msgs/CameraInfo) - Right camera info

## Viewing Images

Use rqt_image_view to visualize the camera streams:

```bash
# View left camera
ros2 run rqt_image_view rqt_image_view /stereo/left/image_raw

# View right camera
ros2 run rqt_image_view rqt_image_view /stereo/right/image_raw
```

## Camera Calibration

To use camera calibration:

1. Calibrate your stereo camera using the camera_calibration package
2. Save the calibration files
3. Update the configuration file with the calibration file paths:

```yaml
left_camera_info_url: "file:///path/to/left_camera.yaml"
right_camera_info_url: "file:///path/to/right_camera.yaml"
```

## Troubleshooting

### Camera not found

Check if the camera is detected:

```bash
lsusb | grep 32e4:9282
v4l2-ctl --list-devices
```

### Permission denied

Ensure your user is in the video group:

```bash
sudo usermod -a -G video $USER
```

Then log out and log back in.

### Wrong camera assignment

If left/right cameras are swapped, modify the udev rules file to swap the KERNEL numbers:

```bash
sudo nano /etc/udev/rules.d/99-stereo-camera.rules
# Swap video2 and video3
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Image format issues

Check supported formats:

```bash
v4l2-ctl --device=/dev/stereo_left --list-formats-ext
```

## License

MIT

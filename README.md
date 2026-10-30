# gelsight_ros2

ROS 2 packages for interfacing with GelSight tactile sensors (e.g., GelSight Mini).

## Packages

- **[`gelsight_driver`](gelsight_driver/)**: Core driver node that automatically discovers attached GelSight sensors, maintains streaming, handles disconnect/reconnect cycles, and publishes tactile images as ROS 2 `sensor_msgs/msg/Image` topics.
- **[`gelsight_bringup`](gelsight_bringup/)**: Launch package providing [`bringup.launch.py`](gelsight_bringup/launch/bringup.launch.py) for starting and configuring the driver.

## Features

- **Automatic Device Discovery**: Scans `/dev/v4l/by-id` and `/sys/class/video4linux` to locate GelSight devices without requiring fixed `/dev/video*` index assignments.
- **Robust Hot-Plugging**: Automatically detects when the sensor is disconnected or re-attached, recovering the video feed without needing to restart the node.
- **Tactile Cropping & Resizing**: Supports GelSight Mini border cropping (1/7 border removal to isolate the active gel surface) and customizable resolution (default: 1280×720 at 30 Hz), or raw native resolution streaming.

## Building

From your ROS 2 workspace root:

```bash
cd ~/ros2_ws
colcon build --packages-select gelsight_driver gelsight_bringup
source install/setup.bash
```

## Usage

### Quick Start

Launch the driver with default settings (1280×720, published on `image_raw`):

```bash
ros2 launch gelsight_bringup bringup.launch.py
```

### Common Configurations

- **Launch with a namespace:**
  ```bash
  ros2 launch gelsight_bringup bringup.launch.py ns:=gelsight
  ```

- **Stream raw native sensor resolution (3280×2464, uncropped):**
  ```bash
  ros2 launch gelsight_bringup bringup.launch.py image_width:=0 image_height:=0 crop:=false
  ```

- **Specify an explicit device path or custom topic:**
  ```bash
  ros2 launch gelsight_bringup bringup.launch.py device_path:=/dev/video6 image_topic:=/gelsight/image_raw
  ```

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `ns` | `""` | Namespace for nodes and topics |
| `device_name` | `"GelSight Mini"` | Device name substring to match for auto-detection |
| `device_path` | `""` | Explicit device path (e.g. `/dev/video6`), overrides auto-detection |
| `device_index` | `"-1"` | Explicit V4L2 device index (e.g. `6`), overrides auto-detection |
| `image_topic` | `"image_raw"` | ROS 2 Image topic name |
| `frame_id` | `"gelsight_frame"` | TF frame ID in message header |
| `image_width` | `"1280"` | Output image width (`<= 0` for native) |
| `image_height` | `"720"` | Output image height (`<= 0` for native) |
| `crop` | `"true"` | Crop borders for GelSight Mini aspect ratio |
| `frame_rate` | `"30.0"` | Target publishing frame rate in Hz |
| `reconnect_interval` | `"2.0"` | Interval in seconds between device probes when disconnected |
| `log_level` | `"info"` | Logging level (`info`, `debug`, `warn`, `error`) |

## License

This project is licensed under the BSD-3-Clause License. See [LICENSE](LICENSE) for details.

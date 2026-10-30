from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context):
    # Load parameters
    log_level = context.launch_configurations["log_level"]
    ns = context.launch_configurations["ns"]
    device_name = context.launch_configurations["device_name"]
    device_path = context.launch_configurations["device_path"]
    device_index = context.launch_configurations["device_index"]
    frame_id = context.launch_configurations["frame_id"]
    image_topic = context.launch_configurations["image_topic"]
    frame_rate = context.launch_configurations["frame_rate"]
    image_width = context.launch_configurations["image_width"]
    image_height = context.launch_configurations["image_height"]
    crop = context.launch_configurations["crop"]
    reconnect_interval = context.launch_configurations["reconnect_interval"]

    # Print parameters
    print("")
    print("Starting GelSight driver with parameters:")
    print(" log_level:           " + log_level)
    if ns == "":
        print(" ns:                  " + "/")
    else:
        print(" ns:                  " + "/" + ns)
    print(" device_name:         " + device_name)
    if device_path != "":
        print(" device_path:         " + device_path)
    if device_index != "-1":
        print(" device_index:        " + device_index)
    print(" frame_id:            " + frame_id)
    print(" image_topic:         " + image_topic)
    print(" frame_rate:          " + frame_rate)
    print(" image_width:         " + image_width)
    print(" image_height:        " + image_height)
    print(" crop:                " + crop)
    print(" reconnect_interval:  " + reconnect_interval)
    print("")

    nodes = []

    nodes.append(
        Node(
            package="gelsight_driver",
            executable="gelsight_driver_node",
            namespace=ns,
            parameters=[
                {
                    "device_name": device_name,
                    "device_path": device_path,
                    "device_index": int(device_index),
                    "frame_id": frame_id,
                    "image_topic": image_topic,
                    "frame_rate": float(frame_rate),
                    "image_width": int(image_width),
                    "image_height": int(image_height),
                    "crop": crop.lower() in ("true", "1"),
                    "reconnect_interval": float(reconnect_interval),
                }
            ],
            arguments=["--ros-args", "--log-level", log_level],
            output="screen",
        )
    )

    return nodes


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "log_level",
            default_value="error",
            description="Log Level to use for all nodes",
            choices=["info", "debug", "error", "warn"],
        ),
        DeclareLaunchArgument(
            "ns",
            default_value="",
            description="Namespace for all nodes",
        ),
        DeclareLaunchArgument(
            "device_name",
            default_value="GelSight Mini",
            description="Device name pattern to match for auto-detection",
        ),
        DeclareLaunchArgument(
            "device_path",
            default_value="",
            description="Explicit device path (e.g. /dev/video6 or /dev/v4l/by-id/...), overrides auto-detection",
        ),
        DeclareLaunchArgument(
            "device_index",
            default_value="-1",
            description="Explicit V4L2 device index (e.g. 0 for /dev/video0), overrides auto-detection",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="gelsight_frame",
            description="TF frame ID stamped in image header",
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="image_raw",
            description="Topic name to publish images on",
        ),
        DeclareLaunchArgument(
            "frame_rate",
            default_value="30.0",
            description="Target publishing frame rate in Hz",
        ),
        DeclareLaunchArgument(
            "image_width",
            default_value="1280",
            description="Output image width (<= 0 for native)",
        ),
        DeclareLaunchArgument(
            "image_height",
            default_value="720",
            description="Output image height (<= 0 for native)",
        ),
        DeclareLaunchArgument(
            "crop",
            default_value="true",
            description="Whether to crop border for aspect ratio matching GelSight Mini specs",
        ),
        DeclareLaunchArgument(
            "reconnect_interval",
            default_value="2.0",
            description="Seconds between checks when sensor is disconnected",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

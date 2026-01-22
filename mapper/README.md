# Mapper Package

## Overview

The **mapper** package is responsible for generating a global map of track boundaries based on local cone perceptions. This global reconstruction is published to be used by the path planning algorithm.

## Nodes

Located inside the `ros` directory, this package contains several ROS2 nodes used for different applications.

### `mapper_node`

The `mapper_node` is the core ROS2 node of this package. It fuses odometry data with local perception data to reconstruct the track globally.

### `odometry_message` and `pose_message`

These nodes are used to convert messages when a specific application requires a different message type. The `odometry_message` node creates and publishes an odometry message based on a subscribed pose message, while the `pose_message` node creates and publishes a pose message based on a subscribed odometry message.

### `sim_track_node`

The `sim_track_node` is an essential ROS2 node used in simulation with FSDS. It converts the complete track from the simulator into a local track in front of the vehicle, simulating the output of the perception pipeline.

## Launcher

There is also a `launch` directory containing all the ROS2 launch files for each of the nodes listed above.

## Running & Launching

```bash
ros2 run mapper mapper_node.py
```

```bash
ros2 launch mapper mapper_launch.py
```

## Dependencies

Core dependencies (minimum):

- ROS 2 Humble (or newer)
- rclcpp / rclpy
- nav_msgs, geometry_msgs, sensor_msgs, lifecycle_msgs, fs_msgs
- tf2 + tf2_ros
- colcon (build system)

## Troubleshooting

- **TF2 Transformations:** The mapper node requires TF2 transformations to function properly. Ensure that the transforms are being received.
- **Timestamp Synchronization:** An `ApproximateTimeSynchronizer` is used with track and odometry messages. Ensure that both topics have synchronized timestamps.

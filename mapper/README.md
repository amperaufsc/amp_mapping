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

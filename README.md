# Mapping and Cone Clustering

This repository contains the modules responsible for environment mapping and cone-based track reconstruction. These components receive perception data, perform spatial clustering using the Luiz López cone clustering algorithm, and generate a structured track map to be consumed by downstream planning and control subsystems.

## The mapping subsystem handles:

Cone detection aggregation from perception inputs
Spatial filtering and outlier rejection
Cone clustering using the Luiz López algorithm
Publishing consistent map representations for planning

##The clustering algorithm (Luiz López method):

Associates detected cones into left and right track boundaries
Uses geometric proximity and heading continuity constraints
Handles partial observations and missing cones
Outputs ordered cone sequences representing track limits

# Clustering Parameters
## Gamma

In the Luiz Lopes cone clustering algorithm, the parameter γ (gamma) acts as a weighting factor that balances spatial proximity and geometric consistency during cluster formation. Practically, γ regulates how strongly the algorithm penalizes associations between cone detections that are close in distance but inconsistent in orientation or expected track structure.

Higher γ values make the clustering more conservative, reducing incorrect associations at the cost of potentially splitting valid clusters. Lower γ values make clustering more permissive, increasing sensitivity but also the risk of forming spurious groupings. Therefore, γ is treated as a tunable hyperparameter adjusted empirically to achieve a trade-off between robustness and responsiveness.

## Covariance

Covariance is used to represent the spatial uncertainty of estimated cone positions within each cluster. Each cluster maintains a covariance matrix describing the dispersion of detections around the estimated cone center.

Mathematically, the covariance encodes both variance along each axis and correlation between axes, allowing the algorithm to infer the principal orientation and confidence of each cluster. Small covariance values indicate high confidence in the cone position estimate, while large covariance values suggest noisy or ambiguous detections.

This uncertainty representation is later exploited by downstream mapping and planning modules to weight cone reliability during track reconstruction.

## System Data Flow (Conceptual)
Perception → Mapping → Path Planning → Vehicle Control → Actuation

Both subsystems interface through ROS 2 topics using consistent timestamping and coordinate frames, enabling integration with upstream modules such as mapping and perception, and downstream modules for actuation.


---

# ROS Interfaces

## Topics (Example)

  self.track_sub = Subscriber(self,TrackStampedWithCovariance,"track")
  self.odom_sub = Subscriber(self,Odometry,"odom")
        
       
        

## Message Time Synchronization
### Approximate Time Synchronizer

To ensure temporal consistency between mapping outputs and vehicle state estimation, this package employs the ApproximateTimeSynchronizer from the ROS 2 message_filters library. This mechanism synchronizes incoming messages from multiple topics based on their timestamps, allowing reliable data association even when exact time alignment is not possible.

In particular, the mapping node synchronizes:


`TrackStampedWithCovariance` – clustered track boundaries with uncertainty information

`Odometry` – vehicle pose and motion state

Since cone clustering and odometry pipelines operate at different update rates and may experience transport latency, strict timestamp equality would lead to frequent message drops. The approximate synchronizer instead matches messages whose timestamps fall within a configurable tolerance window.

This guarantees that the clustering results are always processed together with a coherent vehicle pose estimate, preventing spatial inconsistencies in track reconstruction and improving stability in downstream planning and control modules.

Typical configuration:

Queue size: configurable based on sensor rates

Time tolerance (slop): tuned to match perception and odometry latency

This synchronization strategy ensures robust integration between perception-driven mapping and state estimation in real-time operation.

> Topics and messages used in Mapper package.

---

| Module           | Direction | Topic                     | Message Type                 | Notes |
|------------------|-----------|---------------------------|-------------------------------|-------|
| Mapper     | Sub       | `/odom`                       | `sensor_msgs/Odom`                       | Odometry input |
| Mapper     | Sub       | `/TrackStampedWithCovariance` | `fs_msgs/ConeWithCovariance[] track`     | Track input    |
| Mapper     | Pub       | `/Track`                      | `fs_msgs/Cone[] track`                   | Track output   |


# Dependencies

Core dependencies (minimum):

- ROS 2 Humble (or newer)
- `rclcpp` / `rclpy`
- `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `lifecycle_msgs`, `fs_msgs`
- `tf2` + `tf2_ros`
- `colcon` (build system)

---


## Commands for compiling packages 

### For compiling both, use: 
```bash
    colcon build 
   ```

### For compiling individualy, use: 
```bash
    colcon build --packages-select ros2_mapper
   ```



## Running & Launching

### Mapper launchs: 

```bash
    ros2 run ros2_mapper mapper_node.py
   ```

```bash
    ros2 launch ros2_mapper mapper.launch.py
   ```
## Simulation Track Test Node
### sim_track_node

For controlled testing and debugging of the mapping pipeline, this package provides a dedicated ROS 2 node named `sim_track_node`. This node subscribes to the complete ground-truth track provided by the FSDS simulator and generates a synthetic partial track corresponding to the vehicle’s current camera field of view.

The generated partial track is published as a simulated perception input, enabling evaluation of the clustering and mapping algorithms under deterministic and repeatable conditions.

### Purpose and Features

The `sim_track_node` is primarily intended to isolate and diagnose mapping issues that originate from:

Cone detection uncertainties

Incorrect frame transformations

Inconsistent sensor timestamps

By bypassing real perception pipelines, the node allows focused testing of the clustering and mapping logic alone.

### Functionalities

- Subscribes to the full FSDS track ground truth

- Extracts cones within a configurable camera visibility range

- Publishes a synthetic TrackStampedWithCovariance message

- Optionally injects Gaussian positional noise into cone locations

- Enables robustness testing of clustering and track reconstruction algorithms

### Gaussian Noise Injection

To evaluate algorithm resilience against perception errors, the node can apply zero-mean Gaussian noise to cone positions.



#### Typical Use Cases

- Debugging clustering behavior without perception noise

- Validating coordinate frame transformations

- Stress-testing mapping under increasing uncertainty

- Regression testing after algorithm modifications

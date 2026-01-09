### Localization and Mapping 
This repository contains the mapping-related packages responsible for generating spatial representations of the environment. The mapping stack integrates both visual-based and pose-graph-based SLAM solutions to ensure consistent map reconstruction and localization within a ROS 2 ecosystem.

# The mapping modules used include:
ORB-SLAM for visual SLAM and real-time pose estimation
SLAMToolbox for incremental pose-graph mapping, loop closure, and persistent map handling
A custom mapper algorithm for environment reconstruction and track representation
The mapping layer outputs consistent maps and localization data to downstream modules such as perception, planning, and control. All outputs use ROS 2 timestamping and standard coordinate frames to maintain interoperability across the system.

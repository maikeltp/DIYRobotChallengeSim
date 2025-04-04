# Simulation Package

This package contains simulation-related functionality for the DIY Robot Challange project.

## Running the Simulation

To load the simulation environment with a race track layout and an example vehicle:

```bash
ros2 launch sim load_sim.launch.py
```

### Running gazebo alone
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=<path-to-repo>/install/sim/lib
cd <path-to-repo>/src/sim/worlds
gz sim --verbose -r <path-to-repo>/src/sim/worlds/track_layout2.world
```

To see all the entity components available for the gazebo installation:
```bash
ls /opt/ros/jazzy/opt/gz_sim_vendor/include/gz/sim8/gz/sim/components
```

## Launch Files

- `load_sim.launch.py`: Launches Gazebo and spawns a vehicle model in the simulation environment

## Configuration

- `bridge_config.yaml`: The bridge configuration file manages the communication interface between ROS 2 and the simulation environment.

This configuration file defines:
- Topic mappings between Gazebo and ROS 2
- Message types for each topic

For details on the messages type translations see [ros_gz_bridge](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md)

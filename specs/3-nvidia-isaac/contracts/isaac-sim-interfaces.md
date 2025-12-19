# Isaac Sim USD Interfaces for Educational Module

## Overview
This document outlines the Universal Scene Description (USD) interfaces that students will interact with in the NVIDIA Isaac Sim educational module. USD is the core format used by Isaac Sim for scene representation and robot definitions.

## USD Prim Types Used in Tutorials

### Robot Definition Prims
- **Xform**: Container for robot transforms and hierarchy
- **Joint**: Defines kinematic joints between robot links
- **RigidBody**: Physics properties for robot parts
- **Sensors**: Camera, LiDAR, IMU sensor definitions

### Scene Object Prims
- **Mesh**: Static and dynamic objects in the environment
- **Material**: Surface properties and appearance
- **Light**: Lighting sources and properties
- **PhysicsScene**: Physics simulation parameters

## Key USD Paths and Structures

### Robot Model Structure
```
/World/Robots/{robot_name}
├── /base_link          # Robot base coordinate frame
├── /sensor_0           # First sensor mount point
├── /sensor_1           # Second sensor mount point
└── /links/...          # Robot kinematic chain
```

### Scene Structure
```
/World
├── /Robots/            # All robot instances
├── /Objects/           # Static and dynamic objects
├── /Lights/            # Lighting configuration
└── /Sensors/           # World sensors (not attached to robots)
```

## Isaac Sim API Endpoints for Tutorials

### Scene Management
- **create_scene()**: Initialize a new simulation environment
- **load_scene(usd_path)**: Load a pre-built USD scene
- **reset_scene()**: Reset scene to initial state

### Robot Control
- **spawn_robot(urdf_path, position)**: Add robot to scene
- **set_robot_state(joint_positions)**: Set robot configuration
- **get_robot_state()**: Query current robot state

### Sensor Interface
- **get_sensor_data(sensor_name)**: Retrieve sensor readings
- **set_sensor_parameters(sensor_name, params)**: Configure sensor settings
- **enable_sensor(sensor_name)**: Activate sensor for data collection

### Simulation Control
- **simulate_frame()**: Run one simulation step
- **simulate_time(duration)**: Run simulation for specified time
- **pause_simulation()**: Pause physics simulation
- **resume_simulation()**: Resume physics simulation

## USD File Examples for Educational Use

### Basic Robot USD
```usd
# Example robot definition for educational use
def Xform "Robot" (
    prepend apiSchemas = ["IsaacArticulatedRobot"]
)
{
    # Robot-specific properties and hierarchy
    # To be explored in Chapter 1 tutorials
}
```

### Sensor Configuration USD
```usd
# Example sensor configuration
def Camera "sensor_0" (
    prepend apiSchemas = ["IsaacSensor"]
)
{
    # Camera-specific properties
    # To be configured in Chapter 1 tutorials
}
```

## Common USD Operations in Tutorials

### Creating Objects
1. Define the prim type and name
2. Set transform properties (position, rotation, scale)
3. Add physics and visual properties
4. Connect to scene hierarchy

### Configuring Sensors
1. Add sensor prim to robot or scene
2. Set sensor-specific parameters (FOV, range, etc.)
3. Configure output format and topics
4. Verify sensor placement and orientation

## Validation and Error Handling

### Common USD Validation Checks
- Verify all referenced assets exist
- Check for proper kinematic chain formation
- Validate sensor mounting positions
- Ensure physics properties are consistent

### Error Recovery Strategies
- Provide default configurations when custom USD fails
- Offer step-by-step USD debugging exercises
- Include common error examples and solutions
- Guide students through USD troubleshooting
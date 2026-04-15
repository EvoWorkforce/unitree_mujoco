# Trajectory Configuration Guide

## Overview

The trajectory player now supports runtime configuration via YAML files. You can modify trajectories without rebuilding the simulation!

## Quick Start

1. **Edit the trajectory**: Modify `simulate/trajectory.yaml`
2. **Run the simulation**: `./unitree_mujoco` (in build directory)
3. **Play the trajectory**: Press `Q` in the simulation window

## Configuration File Structure

### Parameters Section
```yaml
parameters:
  max_velocity: 0.2              # m/s - Maximum linear speed
  max_acceleration: 0.4          # m/s^2 - Linear acceleration
  max_angular_velocity: 0.5      # rad/s - Maximum rotation speed
  max_angular_acceleration: 0.3  # rad/s^2 - Angular acceleration
```

### Segments Section
```yaml
segments:
  # Each segment: [x_start, y_start, x_end, y_end, yaw_start, yaw_end]
  - [0.0, 0.0, 2.0, 0.0, 0.0, 0.0]        # Move from (0,0) to (2,0), no rotation
  - [2.0, 0.0, 2.0, 2.0, 0.0, 1.5708]     # Move north, rotate to 90° (π/2)
```

## Coordinate System

- **Position**: (x, y) in meters
- **Yaw**: Angle in radians
  - 0 rad = East (→)
  - π/2 ≈ 1.5708 rad = North (↑)
  - π ≈ 3.14159 rad = West (←)
  - -π/2 ≈ -1.5708 rad = South (↓)

## Common Angle Values

| Degrees | Radians | Direction |
|---------|---------|-----------|
| 0°      | 0.0     | East      |
| 45°     | 0.7854  | NE        |
| 90°     | 1.5708  | North     |
| 135°    | 2.3562  | NW        |
| 180°    | 3.14159 | West      |
| -45°    | -0.7854 | SE        |
| -90°    | -1.5708 | South     |
| -135°   | -2.3562 | SW        |

## Example Trajectories

### Square Path (default)
See `trajectory.yaml` for a 2m x 2m square with 90° turns at each corner.

### Figure-8 Pattern
```bash
cp trajectory_figure8.yaml.example trajectory.yaml
```

### Simple Back-and-Forth
```bash
cp trajectory_simple.yaml.example trajectory.yaml
```

## Creating Custom Trajectories

### Turn in Place
Same start/end position, different yaw:
```yaml
- [1.0, 1.0, 1.0, 1.0, 0.0, 1.5708]  # Rotate 90° at position (1,1)
```

### Straight Line with Constant Heading
Same yaw for start/end:
```yaml
- [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]  # Move 5m east, maintain 0° heading
```

### Arc Motion
Gradual rotation while moving:
```yaml
- [0.0, 0.0, 2.0, 2.0, 0.0, 1.5708]  # Diagonal move with 90° rotation
```

## Tips

1. **Smooth Transitions**: Start yaw of segment N+1 should match end yaw of segment N
2. **Position Continuity**: End position of segment N should match start position of N+1
3. **Speed Tuning**: Lower velocities = smoother motion, higher = faster but less precise
4. **Testing**: Start with low velocities (0.1-0.2 m/s) and increase gradually

## Keyboard Controls

- **Q**: Play/Stop trajectory
- **Space**: Set manual goal with arrow (while held)
- **Right Alt**: Set manual goal position (click to place)
- **Backspace**: Reset simulation

## Troubleshooting

**Trajectory not loading?**
- Check YAML syntax (proper indentation, valid numbers)
- Verify file is named `trajectory.yaml` in `simulate/` directory
- Check console output for error messages

**Robot not following trajectory?**
- Verify segments are continuous (end of one = start of next)
- Check velocity/acceleration values aren't too high
- Ensure yaw angles are in radians, not degrees

**Need to reset to defaults?**
- Delete or rename `trajectory.yaml`
- Simulation will use hardcoded square path as fallback

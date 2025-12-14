# Testing the Path Follower Node

This guide explains how to test the path follower implementation.

## Prerequisites

1. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --packages-select docking_station
source install/setup.bash
```

## Testing Setup

### Option 1: Minimal Test (Path Follower + Path Publisher)

For basic testing without the full system:

**Terminal 1 - Launch path follower and kinematics:**
```bash
ros2 run docking_station path_follower
```

**Terminal 2 - Launch kinematics node:**
```bash
ros2 run docking_station kinematics
```

**Terminal 3 - Set robot state to 'drawing':**
```bash
ros2 topic pub /robot_state std_msgs/msg/String "data: 'drawing'" --once
```

**Terminal 4 - Publish a test path:**
```bash
# Single waypoint test
ros2 run docking_station path_publisher --ros-args \
  -p path_type:=single \
  -p target_x:=0.5 \
  -p target_y:=0.0 \
  -p auto_publish:=true \
  -p delay:=2.0

# Line path test
ros2 run docking_station path_publisher --ros-args \
  -p path_type:=line \
  -p direction:=forward \
  -p length:=0.5 \
  -p num_waypoints:=5 \
  -p auto_publish:=true \
  -p delay:=2.0

# Square path test
ros2 run docking_station path_publisher --ros-args \
  -p path_type:=square \
  -p length:=0.3 \
  -p auto_publish:=true \
  -p delay:=2.0
```

### Option 2: Full System Launch

Launch all nodes (if you have the full system):
```bash
ros2 launch docking_station system_launch.py
```

Then in another terminal, set state and publish path as above.

## Monitoring Topics

**Watch path follower output:**
```bash
ros2 topic echo /drawing/cmd_vel
```

**Monitor progress:**
```bash
ros2 topic echo /drawing/feedback/progress
ros2 topic echo /drawing/feedback/waypoint_index
```

**Check robot pose (if available):**
```bash
ros2 topic echo /robot_pose
```

**Check robot state:**
```bash
ros2 topic echo /robot_state
```

## Testing Without Physical Robot

If you don't have pose feedback, the path follower will still work but won't have position updates. You can:

1. **Mock pose publisher** (create a simple node that publishes `/robot_pose`)
2. **Use dead reckoning** - the path follower will work but accuracy depends on your odometry

## Test Scenarios

### Test 1: Single Waypoint
- Start at (0, 0)
- Target: (0.5, 0)
- Expected: Robot moves forward 0.5m

### Test 2: Lateral Movement
- Start at (0, 0)
- Target: (0, 0.3)
- Expected: Robot moves left 0.3m

### Test 3: Diagonal Movement
- Start at (0, 0)
- Target: (0.3, 0.3)
- Expected: Robot moves diagonally

### Test 4: Multi-Waypoint Path
- Use line path with 5 waypoints
- Expected: Robot follows waypoints sequentially

## Troubleshooting

**Path follower not starting:**
- Check that robot state is set to 'drawing'
- Verify path was published to `/execute_drawing_path`

**No velocity commands:**
- Check that pose feedback is available (`/robot_pose` or `/imu/data`)
- Verify path has waypoints

**Robot not reaching waypoints:**
- Adjust `waypoint_tolerance` parameter (default 0.05m)
- Adjust PID gains (`kp_forward`, `kp_lateral`, `kp_heading`)
- Check max velocity limits

**Velocity commands too aggressive:**
- Reduce PID gains
- Reduce `max_forward_vel`, `max_lateral_vel`, `max_angular_vel`

## Parameter Tuning

Key parameters to adjust:

- `waypoint_tolerance`: How close robot needs to be to waypoint (default: 0.05m)
- `heading_tolerance`: How close heading needs to be (default: 0.1 rad ≈ 5.7°)
- `kp_forward`, `kp_lateral`, `kp_heading`: Proportional gains (start with 2.0, 2.0, 1.0)
- `max_forward_vel`, `max_lateral_vel`: Maximum velocities (default: 0.3 m/s)
- `control_frequency`: Control loop rate (default: 20 Hz)

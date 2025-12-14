#!/bin/bash
# Simple test script for path follower
# Usage: ./test_path_follower.sh [test_type]
# test_type: single, line, square (default: single)

TEST_TYPE=${1:-single}

echo "========================================="
echo "Path Follower Test Script"
echo "========================================="
echo ""
echo "Make sure you have:"
echo "  1. Built the workspace: colcon build --packages-select docking_station"
echo "  2. Sourced the workspace: source install/setup.bash"
echo "  3. Path follower and kinematics nodes running"
echo "  4. Robot state set to 'drawing'"
echo ""
echo "Press Enter to continue or Ctrl+C to cancel..."
read

echo ""
echo "Setting robot state to 'drawing'..."
ros2 topic pub /robot_state std_msgs/msg/String "data: 'drawing'" --once

echo ""
echo "Waiting 1 second..."
sleep 1

echo ""
echo "Publishing $TEST_TYPE path..."
echo ""

case $TEST_TYPE in
  single)
    ros2 run docking_station path_publisher --ros-args \
      -p path_type:=single \
      -p target_x:=0.5 \
      -p target_y:=0.0 \
      -p auto_publish:=true \
      -p delay:=1.0
    ;;
  line)
    ros2 run docking_station path_publisher --ros-args \
      -p path_type:=line \
      -p direction:=forward \
      -p length:=0.5 \
      -p num_waypoints:=5 \
      -p auto_publish:=true \
      -p delay:=1.0
    ;;
  square)
    ros2 run docking_station path_publisher --ros-args \
      -p path_type:=square \
      -p length:=0.3 \
      -p auto_publish:=true \
      -p delay:=1.0
    ;;
  *)
    echo "Unknown test type: $TEST_TYPE"
    echo "Valid types: single, line, square"
    exit 1
    ;;
esac

echo ""
echo "Path published! Monitor topics:"
echo "  ros2 topic echo /drawing/cmd_vel"
echo "  ros2 topic echo /drawing/feedback/progress"
echo ""

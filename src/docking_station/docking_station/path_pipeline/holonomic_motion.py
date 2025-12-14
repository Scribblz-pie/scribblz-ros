"""Holonomic motion generation for Kiwi drive robots.

This module generates wheel commands that exploit the holonomic (omnidirectional)
capabilities of the Kiwi drive. The key insight is that the robot can translate
in any direction WITHOUT rotating - rotation is only needed to avoid collisions.

For drawing curves:
- Primary motion is TRANSLATION along the curve
- Rotation is MINIMAL and only used to keep body clear of drawn ink
- Combined translation+rotation when orientation must change
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple
import math

import numpy as np

from .orientation_planner import OrientedWaypoint, normalize_angle, shortest_angle_diff
from .image_processing import RobotPose, BodyTwist, WheelCommand, compute_kiwi_wheel_speeds


def world_to_body_velocity(
    vx_world: float,
    vy_world: float,
    theta: float,
) -> Tuple[float, float]:
    """Transform world-frame velocity to body-frame velocity.
    
    Args:
        vx_world: X velocity in world frame
        vy_world: Y velocity in world frame
        theta: Robot orientation (radians)
    
    Returns:
        (vx_body, vy_body): Velocity in body frame
    """
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    vx_body = cos_t * vx_world + sin_t * vy_world
    vy_body = -sin_t * vx_world + cos_t * vy_world
    return vx_body, vy_body


@dataclass
class MotionSegment:
    """A segment of motion between two waypoints."""
    
    # Start state
    start_x: float
    start_y: float
    start_theta: float
    
    # End state
    end_x: float
    end_y: float
    end_theta: float
    
    # Motion parameters
    duration: float
    pen_down: bool
    
    # Velocities (world frame, for marker)
    vx_world: float
    vy_world: float
    omega: float
    
    # Stroke tracking
    stroke_index: int
    segment_index: int


def plan_motion_segment(
    wp1: OrientedWaypoint,
    wp2: OrientedWaypoint,
    robot_speed: float,
    turn_speed_rad: float,
    segment_index: int = 0,
) -> MotionSegment:
    """Plan motion from waypoint 1 to waypoint 2.
    
    For holonomic motion:
    - Translation and rotation happen simultaneously
    - Duration is max of translation time and rotation time
    - Velocities are scaled to finish both at same time
    """
    dx = wp2.x - wp1.x
    dy = wp2.y - wp1.y
    distance = math.hypot(dx, dy)
    
    dtheta = shortest_angle_diff(wp1.theta, wp2.theta)
    
    # Compute required times for pure translation and pure rotation
    translation_time = distance / robot_speed if robot_speed > 0 else 0.0
    rotation_time = abs(dtheta) / turn_speed_rad if turn_speed_rad > 0 else 0.0
    
    # Use the longer time - motions happen simultaneously
    duration = max(translation_time, rotation_time, 1e-6)
    
    # Compute velocities to achieve motion in the given duration
    if duration > 1e-9:
        vx_world = dx / duration
        vy_world = dy / duration
        omega = dtheta / duration
    else:
        vx_world = 0.0
        vy_world = 0.0
        omega = 0.0
    
    return MotionSegment(
        start_x=wp1.x,
        start_y=wp1.y,
        start_theta=wp1.theta,
        end_x=wp2.x,
        end_y=wp2.y,
        end_theta=wp2.theta,
        duration=duration,
        pen_down=wp1.pen_down,
        vx_world=vx_world,
        vy_world=vy_world,
        omega=omega,
        stroke_index=wp1.stroke_index,
        segment_index=segment_index,
    )


def motion_segment_to_wheel_command(
    segment: MotionSegment,
    timestamp: float,
    marker_offset_x: float,
    marker_offset_y: float,
    wheelbase_l: float,
) -> WheelCommand:
    """Convert a motion segment to a wheel command.
    
    Handles the kinematics transformation from marker velocity to wheel speeds.
    
    The marker moves at (vx_world, vy_world) in world frame.
    The robot center velocity is computed accounting for the marker offset.
    """
    # Average orientation during the motion
    avg_theta = normalize_angle(segment.start_theta + segment.omega * segment.duration / 2)
    cos_t = math.cos(avg_theta)
    sin_t = math.sin(avg_theta)
    
    # Transform marker offset to world frame
    r_marker_world_x = marker_offset_x * cos_t - marker_offset_y * sin_t
    r_marker_world_y = marker_offset_x * sin_t + marker_offset_y * cos_t
    
    # Robot center velocity from marker velocity
    # v_center = v_marker - omega × r_marker
    # In 2D: omega × r = (-omega * r_y, omega * r_x)
    v_cx_world = segment.vx_world + segment.omega * r_marker_world_y
    v_cy_world = segment.vy_world - segment.omega * r_marker_world_x
    
    # Transform to body frame
    vx_body, vy_body = world_to_body_velocity(v_cx_world, v_cy_world, avg_theta)
    
    # Compute wheel speeds
    wheel_speeds = compute_kiwi_wheel_speeds(vx_body, vy_body, segment.omega, wheelbase_l)
    
    pose = RobotPose(x=segment.start_x, y=segment.start_y, yaw=segment.start_theta)
    body_twist = BodyTwist(vx=vx_body, vy=vy_body, omega=segment.omega, pen_down=segment.pen_down)
    
    return WheelCommand(
        timestamp=timestamp,
        duration=segment.duration,
        pose=pose,
        body_twist=body_twist,
        world_velocity=(v_cx_world, v_cy_world),
        wheel_speeds=tuple(float(w) for w in wheel_speeds),
        pen_down=segment.pen_down,
        polyline_index=segment.stroke_index,
        segment_index=segment.segment_index,
    )


def generate_holonomic_commands(
    waypoints: List[OrientedWaypoint],
    robot_speed: float,
    turn_speed_deg_per_sec: float,
    marker_offset_x: float,
    marker_offset_y: float,
    wheelbase_l: float,
    min_segment_duration: float = 0.01,
    penup_speed: Optional[float] = None,
) -> Tuple[List[WheelCommand], dict]:
    """Generate wheel commands from precomputed waypoints using holonomic motion.
    
    This is the main entry point for motion generation.
    
    Args:
        waypoints: Precomputed waypoints with positions and orientations
        robot_speed: Translation speed when pen is down (m/s)
        turn_speed_deg_per_sec: Rotation speed (deg/s)
        marker_offset_x: Marker X offset in body frame
        marker_offset_y: Marker Y offset in body frame
        wheelbase_l: Robot wheelbase (distance from center to wheel)
        min_segment_duration: Minimum duration for a segment (filters tiny moves)
        penup_speed: Translation speed when pen is up (m/s). If None, uses robot_speed.
    
    Returns:
        (commands, metrics): List of WheelCommands and summary metrics
    """
    if len(waypoints) < 2:
        return [], {"total_duration": 0.0, "total_distance": 0.0, "total_rotation": 0.0}
    
    # Use same speed for pen-up if not specified
    if penup_speed is None:
        penup_speed = robot_speed
    
    turn_speed_rad = math.radians(turn_speed_deg_per_sec)
    
    commands: List[WheelCommand] = []
    time_cursor = 0.0
    
    total_distance = 0.0
    total_rotation = 0.0
    total_pendown_distance = 0.0
    total_penup_distance = 0.0
    
    for i in range(len(waypoints) - 1):
        wp1 = waypoints[i]
        wp2 = waypoints[i + 1]
        
        # Use appropriate speed based on pen state
        current_speed = robot_speed if wp1.pen_down else penup_speed
        
        # Plan the motion segment
        segment = plan_motion_segment(wp1, wp2, current_speed, turn_speed_rad, segment_index=i)
        
        # Skip tiny segments
        if segment.duration < min_segment_duration:
            continue
        
        # Convert to wheel command
        cmd = motion_segment_to_wheel_command(
            segment, time_cursor,
            marker_offset_x, marker_offset_y, wheelbase_l,
        )
        commands.append(cmd)
        
        # Update metrics
        dist = math.hypot(wp2.x - wp1.x, wp2.y - wp1.y)
        rot = abs(shortest_angle_diff(wp1.theta, wp2.theta))
        total_distance += dist
        total_rotation += rot
        
        if wp1.pen_down:
            total_pendown_distance += dist
        else:
            total_penup_distance += dist
        
        time_cursor += segment.duration
    
    metrics = {
        "total_duration": time_cursor,
        "total_distance": total_distance,
        "total_rotation_rad": total_rotation,
        "total_rotation_deg": math.degrees(total_rotation),
        "pendown_distance": total_pendown_distance,
        "penup_distance": total_penup_distance,
        "num_commands": len(commands),
    }
    
    return commands, metrics


def optimize_waypoints_for_pure_translation(
    waypoints: List[OrientedWaypoint],
    rotation_threshold_rad: float = 0.05,
) -> List[OrientedWaypoint]:
    """Post-process waypoints to maximize pure translation motion.
    
    Where possible, maintains constant orientation to avoid unnecessary rotation.
    Only changes orientation when required to avoid collisions.
    
    This is an optional optimization step that can further reduce rotation.
    """
    if len(waypoints) < 2:
        return waypoints
    
    result = [waypoints[0]]
    
    for i in range(1, len(waypoints)):
        wp = waypoints[i]
        prev = result[-1]
        
        # Check if orientation change is small enough to skip
        dtheta = abs(shortest_angle_diff(prev.theta, wp.theta))
        
        if dtheta < rotation_threshold_rad and wp.pen_down == prev.pen_down:
            # Keep previous orientation (pure translation)
            result.append(OrientedWaypoint(
                x=wp.x,
                y=wp.y,
                theta=prev.theta,  # Keep previous orientation
                pen_down=wp.pen_down,
                tangent_x=wp.tangent_x,
                tangent_y=wp.tangent_y,
                stroke_index=wp.stroke_index,
                t_param=wp.t_param,
            ))
        else:
            # Orientation change is necessary
            result.append(wp)
    
    return result


def generate_motion_from_polylines(
    polylines,
    robot_side_length: float,
    robot_speed: float,
    turn_speed_deg_per_sec: float,
    marker_offset_x: float,
    marker_offset_y: float,
    wheelbase_l: float,
    samples_per_stroke: int = 50,
    start_position: Optional[Tuple[float, float]] = None,
) -> Tuple[List[WheelCommand], List[OrientedWaypoint], dict]:
    """High-level function to generate motion commands from polylines.
    
    This combines all steps of the new pipeline:
    1. Stroke ordering (TSP with containment)
    2. Orientation planning (precomputed, tangent-based)
    3. Holonomic motion generation
    
    Returns:
        (commands, waypoints, metrics)
    """
    from .orientation_planner import plan_orientations_from_polylines
    
    # Plan orientations (includes stroke ordering)
    waypoints = plan_orientations_from_polylines(
        polylines,
        robot_side_length,
        marker_offset_x,
        marker_offset_y,
        samples_per_stroke=samples_per_stroke,
        start_position=start_position,
    )
    
    if not waypoints:
        return [], [], {"total_duration": 0.0}
    
    # Optional: optimize for pure translation
    waypoints = optimize_waypoints_for_pure_translation(waypoints)
    
    # Generate wheel commands
    commands, metrics = generate_holonomic_commands(
        waypoints,
        robot_speed,
        turn_speed_deg_per_sec,
        marker_offset_x,
        marker_offset_y,
        wheelbase_l,
    )
    
    return commands, waypoints, metrics


__all__ = [
    "MotionSegment",
    "world_to_body_velocity",
    "plan_motion_segment",
    "motion_segment_to_wheel_command",
    "generate_holonomic_commands",
    "optimize_waypoints_for_pure_translation",
    "generate_motion_from_polylines",
]


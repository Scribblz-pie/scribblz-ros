"""Orientation planner for precomputing robot orientations along drawing paths.

This module precomputes safe robot orientations for the entire path BEFORE simulation,
eliminating per-step collision checking and enabling globally optimal orientation strategies.

Key insight: For a holonomic (Kiwi) drive robot, the optimal orientation keeps the
triangular body "behind" the marker relative to the drawing direction (tangent).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple
import math

import numpy as np

from shapely.geometry import LineString, Polygon, MultiLineString, Point
from shapely.ops import unary_union

from .stroke_ordering import OrderedStroke, Stroke


@dataclass
class OrientedWaypoint:
    """A waypoint with position, orientation, and drawing state."""
    
    x: float
    y: float
    theta: float  # Robot orientation (radians)
    pen_down: bool
    tangent_x: float  # Unit tangent at this point
    tangent_y: float
    stroke_index: int  # Which stroke this belongs to (-1 for pen-up travel)
    t_param: float  # Parameter along stroke [0, 1]
    
    @property
    def position(self) -> Tuple[float, float]:
        return (self.x, self.y)


def normalize_angle(theta: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return (theta + math.pi) % (2 * math.pi) - math.pi


def shortest_angle_diff(from_angle: float, to_angle: float) -> float:
    """Compute shortest angular difference from from_angle to to_angle."""
    return normalize_angle(to_angle - from_angle)


def get_robot_body_polygon(
    marker_x: float,
    marker_y: float,
    theta: float,
    side_length: float,
    marker_offset_x: float,
    marker_offset_y: float,
    buffer_margin: float = 0.0,
) -> Polygon:
    """Compute robot body polygon given marker position and orientation.
    
    The robot is a triangular body with vertices at angles 0, 2π/3, 4π/3 from center.
    The marker is at (marker_offset_x, marker_offset_y) in the body frame.
    """
    robot_radius = side_length / np.sqrt(3.0)
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    
    # Robot center from marker position
    marker_world_offset_x = marker_offset_x * cos_t - marker_offset_y * sin_t
    marker_world_offset_y = marker_offset_x * sin_t + marker_offset_y * cos_t
    center_x = marker_x - marker_world_offset_x
    center_y = marker_y - marker_world_offset_y
    
    # Triangle vertices at 0, 120, 240 degrees from center
    vertices = []
    for angle_offset in [0, 2 * np.pi / 3, 4 * np.pi / 3]:
        local_x = robot_radius * np.cos(angle_offset)
        local_y = robot_radius * np.sin(angle_offset)
        world_x = center_x + local_x * cos_t - local_y * sin_t
        world_y = center_y + local_x * sin_t + local_y * cos_t
        vertices.append((world_x, world_y))
    
    poly = Polygon(vertices)
    if buffer_margin > 0:
        poly = poly.buffer(buffer_margin, cap_style=2, join_style=2)
    
    return poly


def tangent_to_orientation(
    tangent_x: float,
    tangent_y: float,
    offset_angle: float = math.pi,
) -> float:
    """Convert tangent vector to robot orientation.
    
    By default (offset_angle=π), the robot faces opposite to the tangent,
    meaning the body trails behind the marker.
    """
    tangent_angle = math.atan2(tangent_y, tangent_x)
    return normalize_angle(tangent_angle + offset_angle)


def check_orientation_collision(
    x: float,
    y: float,
    theta: float,
    drawn_geometry,
    robot_side_length: float,
    marker_offset_x: float,
    marker_offset_y: float,
    buffer_margin: float = 0.01,
) -> bool:
    """Check if robot body at given position/orientation collides with drawn ink.
    
    Returns True if collision detected.
    """
    if drawn_geometry is None or drawn_geometry.is_empty:
        return False
    
    robot_poly = get_robot_body_polygon(
        x, y, theta,
        robot_side_length,
        marker_offset_x,
        marker_offset_y,
        buffer_margin,
    )
    
    try:
        # Direct intersection check (buffering is done on robot body via buffer_margin)
        intersection = robot_poly.intersection(drawn_geometry)
        if intersection.is_empty:
            return False
        
        # ANY intersection is a collision (strict check)
        if hasattr(intersection, 'area') and intersection.area > 1e-10:
            return True
        if hasattr(intersection, 'length') and intersection.length > 1e-6:
            return True
        
        return False
    except Exception:
        return False


def check_rotation_sweep_collision(
    marker_x: float,
    marker_y: float,
    theta_start: float,
    theta_end: float,
    drawn_geometry,
    robot_side_length: float,
    marker_offset_x: float,
    marker_offset_y: float,
    buffer_margin: float = 0.01,
    num_steps: int = 8,
) -> bool:
    """Check if rotating from theta_start to theta_end causes collision.
    
    CRITICAL: Rotation happens around robot CENTER, not marker.
    The marker and body sweep through an arc during rotation.
    
    Returns True if any intermediate state collides.
    """
    if drawn_geometry is None or drawn_geometry.is_empty:
        return False
    
    delta_theta = shortest_angle_diff(theta_start, theta_end)
    if abs(delta_theta) < 0.01:  # Less than 0.5 degrees, no significant rotation
        return False
    
    # Check intermediate orientations during the rotation sweep
    for i in range(num_steps + 1):
        t = i / num_steps
        theta_interp = normalize_angle(theta_start + t * delta_theta)
        
        # During rotation around center, marker position changes!
        # Compute where the marker would be at this intermediate orientation
        # Given: final marker position (marker_x, marker_y) at theta_end
        # We need marker position at theta_interp
        
        # Actually, for in-place rotation, the CENTER stays fixed.
        # The marker sweeps in an arc around the center.
        # Let's compute the center position from the final marker position
        cos_end = math.cos(theta_end)
        sin_end = math.sin(theta_end)
        # Center = marker - rotated_offset
        marker_world_x = marker_offset_x * cos_end - marker_offset_y * sin_end
        marker_world_y = marker_offset_x * sin_end + marker_offset_y * cos_end
        center_x = marker_x - marker_world_x
        center_y = marker_y - marker_world_y
        
        # Now compute where marker would be at theta_interp
        cos_interp = math.cos(theta_interp)
        sin_interp = math.sin(theta_interp)
        marker_interp_offset_x = marker_offset_x * cos_interp - marker_offset_y * sin_interp
        marker_interp_offset_y = marker_offset_x * sin_interp + marker_offset_y * cos_interp
        marker_interp_x = center_x + marker_interp_offset_x
        marker_interp_y = center_y + marker_interp_offset_y
        
        # Check collision at this intermediate state
        if check_orientation_collision(
            marker_interp_x, marker_interp_y, theta_interp,
            drawn_geometry, robot_side_length,
            marker_offset_x, marker_offset_y, buffer_margin
        ):
            return True
    
    return False


def find_safe_orientation(
    x: float,
    y: float,
    tangent_x: float,
    tangent_y: float,
    drawn_geometry,
    robot_side_length: float,
    marker_offset_x: float,
    marker_offset_y: float,
    num_candidates: int = 36,  # Search every 10 degrees
    buffer_margin: float = 0.025,  # 2.5cm buffer (more conservative)
    current_theta: Optional[float] = None,  # Current orientation (for rotation sweep check)
) -> Tuple[float, bool]:
    """Find a safe orientation at given position.
    
    First tries the tangent-based default orientation (body behind marker).
    If that collides, searches through candidate angles.
    Also checks that ROTATING to the new orientation is safe (sweep check).
    
    Returns:
        (theta, is_safe): Best orientation found and whether it's collision-free
    """
    # Default: body trails marker (opposite to tangent)
    default_theta = tangent_to_orientation(tangent_x, tangent_y, offset_angle=math.pi)
    
    # If no drawn geometry, default is always safe
    if drawn_geometry is None or drawn_geometry.is_empty:
        return default_theta, True
    
    def is_orientation_safe(theta: float) -> bool:
        """Check if orientation is safe AND rotation to it is safe."""
        # Check final position
        if check_orientation_collision(
            x, y, theta, drawn_geometry,
            robot_side_length, marker_offset_x, marker_offset_y, buffer_margin
        ):
            return False
        
        # Check rotation sweep (if we have a current orientation)
        if current_theta is not None and abs(shortest_angle_diff(current_theta, theta)) > 0.05:
            if check_rotation_sweep_collision(
                x, y, current_theta, theta, drawn_geometry,
                robot_side_length, marker_offset_x, marker_offset_y, buffer_margin
            ):
                return False
        
        return True
    
    # Check default orientation
    if is_orientation_safe(default_theta):
        return default_theta, True
    
    # Search for alternative orientations
    best_theta = default_theta
    best_safe = False
    min_deviation = float('inf')
    
    # Search in a spiral pattern outward from default orientation
    for i in range(1, num_candidates + 1):
        angle_step = (2 * math.pi / num_candidates) * i
        
        for sign in [1, -1]:
            candidate_theta = normalize_angle(default_theta + sign * angle_step)
            
            if is_orientation_safe(candidate_theta):
                deviation = angle_step
                if not best_safe or deviation < min_deviation:
                    best_theta = candidate_theta
                    best_safe = True
                    min_deviation = deviation
                    if deviation < math.pi / 4:
                        return best_theta, True
    
    return best_theta, best_safe


def smooth_orientations(
    waypoints: List[OrientedWaypoint],
    max_rate: float = 0.5,  # Max change per waypoint (radians)
) -> List[OrientedWaypoint]:
    """Smooth orientation changes to avoid jerky motion.
    
    Limits the rate of orientation change between consecutive waypoints.
    """
    if len(waypoints) < 2:
        return waypoints
    
    result = [waypoints[0]]
    
    for i in range(1, len(waypoints)):
        wp = waypoints[i]
        prev_theta = result[-1].theta
        
        delta = shortest_angle_diff(prev_theta, wp.theta)
        
        if abs(delta) > max_rate:
            # Limit the change
            clamped_delta = max_rate if delta > 0 else -max_rate
            new_theta = normalize_angle(prev_theta + clamped_delta)
        else:
            new_theta = wp.theta
        
        result.append(OrientedWaypoint(
            x=wp.x,
            y=wp.y,
            theta=new_theta,
            pen_down=wp.pen_down,
            tangent_x=wp.tangent_x,
            tangent_y=wp.tangent_y,
            stroke_index=wp.stroke_index,
            t_param=wp.t_param,
        ))
    
    return result


def find_constant_orientation_for_stroke(
    stroke_line: LineString,
    drawn_geometry,
    robot_side_length: float,
    marker_offset_x: float,
    marker_offset_y: float,
    num_test_points: int = 30,  # More test points for thorough checking
    num_angle_candidates: int = 180,  # Every 5 degrees
    buffer_margin: float = 0.025,  # 2.5cm buffer (more conservative)
    preferred_theta: Optional[float] = None,  # Prefer orientations close to this
) -> Optional[float]:
    """Try to find a single orientation that works for the entire stroke.
    
    KEY INSIGHT: At each test point, we only need to avoid:
    - Previously drawn strokes (full)
    - The portion of THIS stroke already drawn (0 to current point)
    
    We CAN be over future parts of the stroke - that's fine!
    
    If preferred_theta is provided, orientations closer to it are tried first.
    
    Returns the orientation if found, None if no single orientation works.
    """
    total_length = stroke_line.length
    if total_length < 1e-6:
        return None
    
    # Sample test points along the stroke with their progress
    test_data = []  # (x, y, portion_drawn)
    for i in range(num_test_points):
        t = i / (num_test_points - 1) if num_test_points > 1 else 0.0
        pt = stroke_line.interpolate(t * total_length)
        # The portion of stroke drawn at this point (0 to t)
        if t > 0.01:
            drawn_portion = LineString([
                stroke_line.interpolate(0).coords[0],
                *[stroke_line.interpolate(s * total_length).coords[0] 
                  for s in [t * j / 10 for j in range(1, 11)]]
            ])
        else:
            drawn_portion = None
        test_data.append((pt.x, pt.y, drawn_portion))
    
    # Generate candidate orientations
    candidates = [(2 * math.pi * i) / num_angle_candidates for i in range(num_angle_candidates)]
    
    # Sort by distance from preferred_theta if provided
    if preferred_theta is not None:
        candidates.sort(key=lambda theta: abs(shortest_angle_diff(preferred_theta, theta)))
    
    # Try each candidate orientation (prioritizing those close to preferred)
    for theta in candidates:
        works_for_all = True
        for x, y, drawn_portion in test_data:
            # Build geometry to check: previous strokes + portion already drawn
            if drawn_geometry is not None and drawn_portion is not None:
                check_geom = unary_union([drawn_geometry, drawn_portion])
            elif drawn_geometry is not None:
                check_geom = drawn_geometry
            elif drawn_portion is not None:
                check_geom = drawn_portion
            else:
                check_geom = None
            
            if check_geom is not None and check_orientation_collision(
                x, y, theta, check_geom,
                robot_side_length, marker_offset_x, marker_offset_y,
                buffer_margin=buffer_margin,
            ):
                works_for_all = False
                break
        
        if works_for_all:
            return theta
    
    return None


def plan_stroke_orientations(
    stroke: Stroke,
    draw_reversed: bool,
    drawn_geometry,
    robot_side_length: float,
    marker_offset_x: float,
    marker_offset_y: float,
    num_samples: int = 50,
    collision_buffer: float = 0.01,
    incoming_theta: Optional[float] = None,  # Orientation from previous pen-up travel
) -> Tuple[List[OrientedWaypoint], any]:
    """Plan orientations for a single stroke.
    
    HOLONOMIC STRATEGY: Try to use CONSTANT orientation for the entire stroke
    (pure translation, no rotation). Only fall back to variable orientation
    if no single orientation works for the whole stroke.
    
    Args:
        incoming_theta: The orientation the robot has when arriving at this stroke.
                       Used to check if rotation to stroke orientation is safe.
    
    Returns:
        (waypoints, updated_geometry): Waypoints and the geometry including this stroke
    """
    coords = list(stroke.geometry.coords)
    if draw_reversed:
        coords = coords[::-1]
    
    if len(coords) < 2:
        return [], drawn_geometry
    
    ls = LineString(coords)
    total_length = ls.length
    
    if total_length < 1e-6:
        return [], drawn_geometry
    
    # Get start position for rotation check
    start_x, start_y = coords[0]
    
    # FIRST: Try to find a constant orientation for the entire stroke
    # This enables pure translation (no rotation during drawing)
    # Always prefer incoming_theta to minimize rotations
    constant_theta = find_constant_orientation_for_stroke(
        ls, drawn_geometry,
        robot_side_length, marker_offset_x, marker_offset_y,
        buffer_margin=collision_buffer,
        preferred_theta=incoming_theta,  # Prefer current orientation to avoid rotation
    )
    
    # If we found a constant orientation, check if rotation is needed and safe
    if constant_theta is not None and incoming_theta is not None:
        # If constant_theta is very close to incoming_theta, no rotation needed
        angle_diff = abs(shortest_angle_diff(incoming_theta, constant_theta))
        if angle_diff < 0.05:  # Less than ~3 degrees, use incoming directly
            constant_theta = incoming_theta
        else:
            # Check if rotating TO constant_theta at stroke start would erase ink
            rotation_causes_erasure = check_rotation_sweep_collision(
                start_x, start_y, incoming_theta, constant_theta, drawn_geometry,
                robot_side_length, marker_offset_x, marker_offset_y, collision_buffer,
            )
            if rotation_causes_erasure:
                # Rotation would erase - try to find a closer orientation
                constant_theta = find_constant_orientation_for_stroke(
                    ls, drawn_geometry,
                    robot_side_length, marker_offset_x, marker_offset_y,
                    buffer_margin=collision_buffer,
                    preferred_theta=incoming_theta,
                )
                # If still causes erasure or not found, just use incoming (no rotation)
                if constant_theta is None:
                    constant_theta = incoming_theta
                else:
                    angle_diff = abs(shortest_angle_diff(incoming_theta, constant_theta))
                    if angle_diff > 0.05:
                        rotation_causes_erasure = check_rotation_sweep_collision(
                            start_x, start_y, incoming_theta, constant_theta, drawn_geometry,
                            robot_side_length, marker_offset_x, marker_offset_y, collision_buffer,
                        )
                        if rotation_causes_erasure:
                            constant_theta = incoming_theta  # Use incoming, no rotation
    
    waypoints = []
    current_theta = constant_theta  # May be None if no constant orientation works
    current_ink = drawn_geometry  # Track incrementally: previous + already-drawn portion
    last_point = None
    
    for i in range(num_samples):
        t = i / (num_samples - 1) if num_samples > 1 else 0.0
        distance_along = t * total_length
        
        point = ls.interpolate(distance_along)
        x, y = point.x, point.y
        
        # Update ink with segment from last point (incremental tracking)
        if last_point is not None and i > 0:
            segment = LineString([last_point, (x, y)])
            if segment.length > 1e-6:
                if current_ink is None:
                    current_ink = segment
                else:
                    current_ink = unary_union([current_ink, segment])
        
        # Compute tangent for reference
        epsilon = min(0.001, total_length * 0.01)
        if distance_along + epsilon <= total_length:
            point_ahead = ls.interpolate(distance_along + epsilon)
            tangent_x = point_ahead.x - x
            tangent_y = point_ahead.y - y
        else:
            point_back = ls.interpolate(max(0, distance_along - epsilon))
            tangent_x = x - point_back.x
            tangent_y = y - point_back.y
        
        mag = math.hypot(tangent_x, tangent_y)
        if mag > 1e-10:
            tangent_x /= mag
            tangent_y /= mag
        else:
            tangent_x, tangent_y = 1.0, 0.0
        
        # Use constant orientation if we found one, otherwise find safe orientation
        if constant_theta is not None:
            theta = constant_theta
        else:
            # Fallback: check against INCREMENTAL geometry (previous + already drawn)
            # This allows body to be over FUTURE parts of stroke
            if current_theta is not None and not check_orientation_collision(
                x, y, current_theta, current_ink,
                robot_side_length, marker_offset_x, marker_offset_y,
                buffer_margin=collision_buffer,
            ):
                theta = current_theta
            else:
                # Need to find a new safe orientation (include rotation sweep check)
                theta, _ = find_safe_orientation(
                    x, y, tangent_x, tangent_y, current_ink,
                    robot_side_length, marker_offset_x, marker_offset_y,
                    buffer_margin=collision_buffer,
                    current_theta=current_theta,  # For rotation sweep check
                )
                current_theta = theta
        
        waypoints.append(OrientedWaypoint(
            x=x, y=y, theta=theta, pen_down=True,
            tangent_x=tangent_x, tangent_y=tangent_y,
            stroke_index=stroke.index, t_param=t,
        ))
        
        last_point = (x, y)
    
    # Return updated geometry including this stroke
    return waypoints, current_ink


def plan_penup_travel(
    start: OrientedWaypoint,
    end_position: Tuple[float, float],
    end_tangent: Tuple[float, float],
    drawn_geometry,
    robot_side_length: float,
    marker_offset_x: float,
    marker_offset_y: float,
    num_samples: int = 10,
) -> List[OrientedWaypoint]:
    """Plan pen-up travel from current position to next stroke start.
    
    Strategy: TRANSLATE FIRST, ROTATE LATER.
    - Keep the current orientation until the robot body is clear of all ink
    - Only then rotate if needed for the next stroke
    - This prevents erasure at stroke ends
    """
    start_x, start_y = start.x, start.y
    end_x, end_y = end_position
    
    travel_dx = end_x - start_x
    travel_dy = end_y - start_y
    travel_dist = math.hypot(travel_dx, travel_dy)
    
    if travel_dist < 1e-6:
        return []
    
    travel_tangent_x = travel_dx / travel_dist
    travel_tangent_y = travel_dy / travel_dist
    
    waypoints = []
    current_theta = start.theta  # Start with current orientation
    has_cleared_ink = False  # Track when we're clear of ink
    
    for i in range(num_samples):
        t = i / (num_samples - 1) if num_samples > 1 else 0.0
        x = start_x + t * travel_dx
        y = start_y + t * travel_dy
        
        # Check if robot body collides with ink at current orientation
        has_collision = check_orientation_collision(
            x, y, current_theta, drawn_geometry,
            robot_side_length, marker_offset_x, marker_offset_y,
            buffer_margin=0.015,
        )
        
        if not has_collision:
            # We're clear of ink now!
            has_cleared_ink = True
        
        if has_collision and not has_cleared_ink:
            # Still over ink from previous stroke - DO NOT ROTATE
            # Just keep translating with same orientation
            pass  # current_theta stays the same
        elif has_collision and has_cleared_ink:
            # Collision with NEW ink (approaching next stroke area)
            # We can safely rotate since we already cleared previous ink
            theta, _ = find_safe_orientation(
                x, y, travel_tangent_x, travel_tangent_y, drawn_geometry,
                robot_side_length, marker_offset_x, marker_offset_y,
                current_theta=current_theta,
            )
            current_theta = theta
        # else: no collision, keep current orientation
        
        waypoints.append(OrientedWaypoint(
            x=x, y=y, theta=current_theta, pen_down=False,
            tangent_x=travel_tangent_x, tangent_y=travel_tangent_y,
            stroke_index=-1, t_param=t,
        ))
    
    return waypoints


def plan_all_orientations(
    ordered_strokes: List[OrderedStroke],
    robot_side_length: float,
    marker_offset_x: float,
    marker_offset_y: float,
    samples_per_stroke: int = 50,
    penup_samples: int = 10,
    collision_buffer: float = 0.01,
    smooth_rate: Optional[float] = 0.3,
    dock_position: Optional[Tuple[float, float]] = None,
    dock_approach: Optional[Tuple[float, float]] = None,
    dock_orientation: float = 0.0,
) -> Tuple[List[OrientedWaypoint], List[OrientedWaypoint], List[OrientedWaypoint]]:
    """Plan orientations for entire drawing path, split into phases.
    
    This is the main entry point for orientation planning.
    Returns three separate waypoint lists for ROS state machine integration.
    
    Args:
        ordered_strokes: Strokes in drawing order with direction
        robot_side_length: Side length of triangular robot
        marker_offset_x: Marker X offset in body frame
        marker_offset_y: Marker Y offset in body frame
        samples_per_stroke: Number of waypoints per stroke
        penup_samples: Number of waypoints for pen-up travel
        collision_buffer: Safety margin for collision detection
        smooth_rate: Max orientation change rate (None to disable smoothing)
        dock_position: Actual dock location (robot starts/ends here)
        dock_approach: Approach point above dock (robot rises to here before traveling)
        dock_orientation: Robot orientation at the docking station
    
    Returns:
        Tuple of (undock_waypoints, draw_waypoints, dock_waypoints):
        - undock_waypoints: Pen-up travel from dock to first stroke
        - draw_waypoints: All drawing (pen-down + inter-stroke pen-up)
        - dock_waypoints: Pen-up travel from last stroke back to dock
    """
    undock_waypoints: List[OrientedWaypoint] = []
    draw_waypoints: List[OrientedWaypoint] = []
    dock_waypoints: List[OrientedWaypoint] = []
    
    if not ordered_strokes:
        return undock_waypoints, draw_waypoints, dock_waypoints
    
    drawn_geometry = None  # Union of all drawn strokes so far
    last_waypoint = None  # Track last waypoint for continuity
    
    # Use approach point if provided, otherwise use dock_position directly
    travel_start = dock_approach if dock_approach else dock_position
    
    # Phase 1: UNDOCK - travel from dock → approach → first stroke
    if dock_position is not None:
        first_stroke = ordered_strokes[0]
        first_start = first_stroke.draw_start
        
        # Compute tangent at start of first stroke
        coords = list(first_stroke.stroke.geometry.coords)
        if first_stroke.draw_reversed:
            coords = coords[::-1]
        if len(coords) >= 2:
            dx = coords[1][0] - coords[0][0]
            dy = coords[1][1] - coords[0][1]
            mag = math.hypot(dx, dy)
            first_tangent = (dx / mag, dy / mag) if mag > 1e-10 else (1.0, 0.0)
        else:
            first_tangent = (1.0, 0.0)
        
        # Step 1: Start at dock position
        dock_wp = OrientedWaypoint(
            x=dock_position[0], y=dock_position[1], theta=dock_orientation,
            pen_down=False, tangent_x=0.0, tangent_y=1.0,  # Pointing up
            stroke_index=-1, t_param=0.0,
        )
        undock_waypoints.append(dock_wp)
        
        # Step 2: Rise up to approach point (if different from dock)
        if dock_approach and dock_approach != dock_position:
            approach_wp = OrientedWaypoint(
                x=dock_approach[0], y=dock_approach[1], theta=dock_orientation,
                pen_down=False, tangent_x=0.0, tangent_y=1.0,
                stroke_index=-1, t_param=0.0,
            )
            # Interpolate from dock to approach
            dock_to_approach = plan_penup_travel(
                dock_wp, dock_approach, (0.0, 1.0),  # Moving up
                None, robot_side_length, marker_offset_x, marker_offset_y,
                num_samples=5,  # Short travel, few samples
            )
            undock_waypoints.extend(dock_to_approach)
            last_undock_wp = undock_waypoints[-1]
        else:
            last_undock_wp = dock_wp
        
        # Step 3: Travel from approach to first stroke
        dist = math.hypot(first_start[0] - last_undock_wp.x, first_start[1] - last_undock_wp.y)
        if dist > 1e-6:
            approach_to_first = plan_penup_travel(
                last_undock_wp, first_start, first_tangent,
                None,  # No drawn geometry yet
                robot_side_length, marker_offset_x, marker_offset_y,
                num_samples=penup_samples,
            )
            undock_waypoints.extend(approach_to_first)
        
        last_waypoint = undock_waypoints[-1] if undock_waypoints else dock_wp
    
    # Phase 2: DRAW - all strokes and inter-stroke travel
    for i, ordered_stroke in enumerate(ordered_strokes):
        stroke = ordered_stroke.stroke
        
        # Plan pen-up travel from previous position
        if i > 0 and (draw_waypoints or last_waypoint):
            prev_wp = draw_waypoints[-1] if draw_waypoints else last_waypoint
            next_start = ordered_stroke.draw_start
            
            # Compute tangent at start of next stroke
            coords = list(stroke.geometry.coords)
            if ordered_stroke.draw_reversed:
                coords = coords[::-1]
            
            if len(coords) >= 2:
                dx = coords[1][0] - coords[0][0]
                dy = coords[1][1] - coords[0][1]
                mag = math.hypot(dx, dy)
                next_tangent = (dx / mag, dy / mag) if mag > 1e-10 else (1.0, 0.0)
            else:
                next_tangent = (1.0, 0.0)
            
            # Check if travel is needed
            dist = math.hypot(next_start[0] - prev_wp.x, next_start[1] - prev_wp.y)
            if dist > 1e-6:
                penup_wps = plan_penup_travel(
                    prev_wp, next_start, next_tangent,
                    drawn_geometry,
                    robot_side_length, marker_offset_x, marker_offset_y,
                    num_samples=penup_samples,
                )
                draw_waypoints.extend(penup_wps)
        
        # Plan orientations for this stroke
        if draw_waypoints:
            incoming_theta = draw_waypoints[-1].theta
        elif last_waypoint:
            incoming_theta = last_waypoint.theta
        else:
            incoming_theta = dock_orientation
            
        stroke_wps, drawn_geometry = plan_stroke_orientations(
            stroke, ordered_stroke.draw_reversed,
            drawn_geometry,
            robot_side_length, marker_offset_x, marker_offset_y,
            num_samples=samples_per_stroke,
            collision_buffer=collision_buffer,
            incoming_theta=incoming_theta,
        )
        draw_waypoints.extend(stroke_wps)
    
    # Phase 3: DOCK - travel from last stroke → approach → dock
    if dock_position is not None and draw_waypoints:
        last_wp = draw_waypoints[-1]
        
        # Step 1: Travel from last stroke to approach point
        approach_target = dock_approach if dock_approach else dock_position
        dist_to_approach = math.hypot(approach_target[0] - last_wp.x, approach_target[1] - last_wp.y)
        
        if dist_to_approach > 1e-6:
            dx = approach_target[0] - last_wp.x
            dy = approach_target[1] - last_wp.y
            mag = math.hypot(dx, dy)
            approach_tangent = (dx / mag, dy / mag)
            
            to_approach_wps = plan_penup_travel(
                last_wp, approach_target, approach_tangent,
                drawn_geometry,  # Full drawn geometry for collision avoidance
                robot_side_length, marker_offset_x, marker_offset_y,
                num_samples=penup_samples,
            )
            dock_waypoints.extend(to_approach_wps)
        
        # Step 2: Drop down from approach to dock (if different)
        if dock_approach and dock_approach != dock_position:
            last_dock_wp = dock_waypoints[-1] if dock_waypoints else last_wp
            
            # Interpolate from approach to dock (moving down)
            approach_to_dock = plan_penup_travel(
                last_dock_wp, dock_position, (0.0, -1.0),  # Moving down
                drawn_geometry,
                robot_side_length, marker_offset_x, marker_offset_y,
                num_samples=5,  # Short travel, few samples
            )
            dock_waypoints.extend(approach_to_dock)
        
        # Final waypoint at dock with dock orientation
        dock_waypoints.append(OrientedWaypoint(
            x=dock_position[0], y=dock_position[1], theta=dock_orientation,
            pen_down=False, tangent_x=0.0, tangent_y=-1.0,  # Pointing down (into dock)
            stroke_index=-1, t_param=1.0,
        ))
    
    # Apply smoothing to each phase separately
    if smooth_rate is not None and smooth_rate > 0:
        undock_waypoints = smooth_orientations(undock_waypoints, max_rate=smooth_rate)
        draw_waypoints = smooth_orientations(draw_waypoints, max_rate=smooth_rate)
        dock_waypoints = smooth_orientations(dock_waypoints, max_rate=smooth_rate)
    
    return undock_waypoints, draw_waypoints, dock_waypoints


def plan_orientations_from_polylines(
    polylines: Sequence[Sequence[Tuple[float, float]]],
    robot_side_length: float,
    marker_offset_x: float,
    marker_offset_y: float,
    samples_per_stroke: int = 50,
    start_position: Optional[Tuple[float, float]] = None,
) -> Tuple[List[OrientedWaypoint], List[OrientedWaypoint], List[OrientedWaypoint]]:
    """Convenience function to plan orientations directly from polylines.
    
    Combines stroke creation, TSP ordering, and orientation planning.
    
    Returns:
        Tuple of (undock_waypoints, draw_waypoints, dock_waypoints)
    """
    from .stroke_ordering import strokes_from_polylines, order_strokes_with_containment
    
    strokes = strokes_from_polylines(polylines)
    if not strokes:
        return [], [], []
    
    ordered = order_strokes_with_containment(strokes, start_position=start_position)
    
    return plan_all_orientations(
        ordered,
        robot_side_length,
        marker_offset_x,
        marker_offset_y,
        samples_per_stroke=samples_per_stroke,
        dock_position=start_position,
    )


__all__ = [
    "OrientedWaypoint",
    "normalize_angle",
    "shortest_angle_diff",
    "get_robot_body_polygon",
    "tangent_to_orientation",
    "check_orientation_collision",
    "find_safe_orientation",
    "smooth_orientations",
    "plan_stroke_orientations",
    "plan_penup_travel",
    "plan_all_orientations",
    "plan_orientations_from_polylines",
]


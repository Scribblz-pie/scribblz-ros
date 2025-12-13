"""Drawing robot path planner with holonomic motion.

Usage examples:
    python path_sim_graph_eulerian.py image.png                    # run full simulation
    python path_sim_graph_eulerian.py image.png --view image       # show image→mask→skeleton
    python path_sim_graph_eulerian.py image.png --view polylines   # show extracted polylines
    python path_sim_graph_eulerian.py image.png --no-animate       # generate commands without animation

Pipeline: image → skeleton → polylines → strokes → orientations → wheel commands
"""

from __future__ import annotations

import argparse
import json
import math
from typing import List, Tuple

import numpy as np

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from shapely.geometry import LineString, MultiLineString
from shapely.ops import unary_union

from path_pipeline.image_processing import (
    DEDUPLICATION_DISTANCE_TOLERANCE_DEFAULT,
    ImageStages,
    generate_image_stages,
    visualize_image_stages,
)
from path_pipeline.stroke_ordering import (
    strokes_from_polylines,
    order_strokes_with_containment,
    compute_total_travel,
)
from path_pipeline.orientation_planner import (
    OrientedWaypoint,
    plan_all_orientations,
    get_robot_body_polygon,
    normalize_angle,
)
from path_pipeline.holonomic_motion import (
    generate_holonomic_commands,
    optimize_waypoints_for_pure_translation,
)


# =========================
# Robot Parameters (meters)
# =========================
ROBOT_SIDE_LENGTH = 0.1732  # Side length of the triangular robot chassis
ROBOT_WHEELBASE_L = ROBOT_SIDE_LENGTH / np.sqrt(3.0)
ROBOT_RADIUS = ROBOT_WHEELBASE_L

# Marker offset in body frame (1/3 down right edge from front vertex)
MARKER_OFFSET_X = ROBOT_RADIUS / 2.0
MARKER_OFFSET_Y = -ROBOT_RADIUS / np.sqrt(3.0)

# =========================
# Canvas Parameters
# =========================
TARGET_CANVAS_WIDTH = 1.0  # Width of drawing area in meters
CANVAS_PADDING = 0.05

# Docking station (bottom-right, robot approaches from above)
DOCK_POSITION = (1.05, -0.15)      # Actual dock location
DOCK_APPROACH = (1.05, 0.05)       # Approach point (directly above dock)
DOCK_ORIENTATION = 0.0             # Robot faces +X at dock

# =========================
# Motion Parameters
# =========================
ROBOT_SPEED = 0.0245  # m/s (pen-down drawing speed)
ROBOT_PENUP_SPEED = 0.0245  # m/s (pen-up travel speed)
ROBOT_TURN_SPEED_DEG_PER_SEC = 110
ERASE_MARGIN = 0.02  # 2cm buffer for collision avoidance

# =========================
# Animation Parameters
# =========================
ANIMATION_FPS = 30
ANIMATION_INTERVAL_MS = 1000 // ANIMATION_FPS
ACTIVE_ANIMATIONS: List[animation.FuncAnimation] = []


# =========================
# Geometry helpers
# =========================
def line_length(geom) -> float:
    """Get total length of a geometry (LineString or MultiLineString)."""
    if geom is None or geom.is_empty:
        return 0.0
    if isinstance(geom, LineString):
        return geom.length
    if isinstance(geom, MultiLineString):
        return sum(g.length for g in geom.geoms)
    try:
        return geom.length
    except Exception:
        return 0.0


def geometry_to_segments(geom) -> List:
    """Convert geometry to list of line segments for plotting."""
    segments = []
    if geom is None or geom.is_empty:
        return segments
    if isinstance(geom, LineString):
        coords = list(geom.coords)
        for i in range(len(coords) - 1):
            segments.append([coords[i], coords[i + 1]])
    elif isinstance(geom, MultiLineString):
        for line in geom.geoms:
            coords = list(line.coords)
            for i in range(len(coords) - 1):
                segments.append([coords[i], coords[i + 1]])
    return segments


def get_robot_body(marker_x: float, marker_y: float, theta: float) -> object:
    """Get robot body polygon with ERASE_MARGIN buffer."""
    return get_robot_body_polygon(
        marker_x, marker_y, theta,
        ROBOT_SIDE_LENGTH,
        MARKER_OFFSET_X,
        MARKER_OFFSET_Y,
        buffer_margin=ERASE_MARGIN,
    )


def waypoints_to_dict_list(waypoints: List[OrientedWaypoint]) -> List[dict]:
    """Convert waypoints to JSON-serializable list."""
    return [
        {
            "x": wp.x,
            "y": wp.y,
            "yaw": wp.theta,
            "pen_down": wp.pen_down,
        }
        for wp in waypoints
    ]


def save_phased_waypoints(
    undock_wps: List[OrientedWaypoint],
    draw_wps: List[OrientedWaypoint],
    dock_wps: List[OrientedWaypoint],
    filepath: str,
    metadata: dict,
) -> None:
    """Save waypoints in phase-separated format for ROS state machine.
    
    Output format:
    {
        "metadata": {...},
        "undock_waypoints": [...],
        "draw_waypoints": [...],
        "dock_waypoints": [...]
    }
    """
    output = {
        "metadata": metadata,
        "undock_waypoints": waypoints_to_dict_list(undock_wps),
        "draw_waypoints": waypoints_to_dict_list(draw_wps),
        "dock_waypoints": waypoints_to_dict_list(dock_wps),
    }
    
    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2)


# =========================
# Pipeline
# =========================
def build_image_stages(image_path: str, dedup_tol: float) -> ImageStages:
    """Load image and extract polylines (always uses cv2 + skeletonization)."""
    return generate_image_stages(
        image_path=image_path,
        target_width=TARGET_CANVAS_WIDTH,
        padding=CANVAS_PADDING,
        smooth_factor=0.0,
        smoothing_points=200,
        simplification_epsilon_factor=0.0,
        dedup_tolerance=dedup_tol,
        extractor="cv2",
        approx_tol=0.0,
        use_skeleton=True,  # Always use skeletonization
    )


def run_pipeline(args, stages: ImageStages):
    """Main pipeline: TSP ordering → precomputed orientations → holonomic motion."""
    polylines = stages.rescaled_polylines
    
    print("Planning stroke order (TSP with containment)...")
    strokes = strokes_from_polylines(polylines)
    if not strokes:
        print("No strokes found.")
        return
    
    ordered_strokes = order_strokes_with_containment(strokes, start_position=DOCK_APPROACH)
    travel_dist = compute_total_travel(ordered_strokes)
    print(f"  {len(ordered_strokes)} strokes, pen-up travel: {travel_dist:.4f}m")
    
    print("Computing orientations (3 phases: undock → draw → dock)...")
    undock_wps, draw_wps, dock_wps = plan_all_orientations(
        ordered_strokes,
        robot_side_length=ROBOT_SIDE_LENGTH,
        marker_offset_x=MARKER_OFFSET_X,
        marker_offset_y=MARKER_OFFSET_Y,
        samples_per_stroke=args.samples_per_stroke,
        collision_buffer=ERASE_MARGIN,
        smooth_rate=0.3,
        dock_position=DOCK_POSITION,
        dock_approach=DOCK_APPROACH,
        dock_orientation=DOCK_ORIENTATION,
    )
    print(f"  Undock: {len(undock_wps)} waypoints")
    print(f"  Draw: {len(draw_wps)} waypoints")
    print(f"  Dock: {len(dock_wps)} waypoints")
    
    # Optimize each phase for pure translation
    undock_wps = optimize_waypoints_for_pure_translation(undock_wps, rotation_threshold_rad=0.05)
    draw_wps = optimize_waypoints_for_pure_translation(draw_wps, rotation_threshold_rad=0.05)
    dock_wps = optimize_waypoints_for_pure_translation(dock_wps, rotation_threshold_rad=0.05)
    
    # Combine all waypoints for command generation and animation
    all_waypoints = undock_wps + draw_wps + dock_wps
    
    print("Generating wheel commands...")
    commands, metrics = generate_holonomic_commands(
        all_waypoints,
        robot_speed=ROBOT_SPEED,
        turn_speed_deg_per_sec=ROBOT_TURN_SPEED_DEG_PER_SEC,
        marker_offset_x=MARKER_OFFSET_X,
        marker_offset_y=MARKER_OFFSET_Y,
        wheelbase_l=ROBOT_WHEELBASE_L,
        penup_speed=ROBOT_PENUP_SPEED,
    )
    
    print("Motion metrics:")
    for key, value in metrics.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")
    
    # Save with phase-separated structure
    metadata = {
        "robot_side_length": ROBOT_SIDE_LENGTH,
        "robot_radius_l": ROBOT_RADIUS,
        "robot_speed": ROBOT_SPEED,
        "robot_turn_speed_deg_per_sec": ROBOT_TURN_SPEED_DEG_PER_SEC,
        "marker_offset_x": MARKER_OFFSET_X,
        "marker_offset_y": MARKER_OFFSET_Y,
        "dock_position": DOCK_POSITION,
            "dock_approach": DOCK_APPROACH,
    }
    save_phased_waypoints(
        undock_wps, draw_wps, dock_wps,
        "waypoints.json", metadata
    )
    print("Saved waypoints to waypoints.json (with undock/draw/dock phases)")
    
    if not args.no_animate:
        print("Simulating for animation...")
        sim_states, erasure_metrics = simulate_for_animation(all_waypoints)
        
        print(f"  Erasure: drawn={erasure_metrics['total_drawn']:.4f}m, "
              f"erased={erasure_metrics['total_erased']:.4f}m, "
              f"ratio={erasure_metrics['erasure_ratio']*100:.1f}%")
        
        # Build target geometry for reference
        target_geom = None
        for os in ordered_strokes:
            geom = os.ordered_geometry
            target_geom = geom if target_geom is None else unary_union([target_geom, geom])
        
        anim_metrics = {
            "drawn_length": metrics.get("pendown_distance", 0.0),
            "penup_travel_length": metrics.get("penup_distance", 0.0),
            "erased_length": erasure_metrics["total_erased"],
            "total_duration_sec": metrics.get("total_duration", 0.0),
        }
        
        print("Starting animation...")
        animate(target_geom, sim_states, anim_metrics, stages.canvas_height)


def simulate_for_animation(
    waypoints: List[OrientedWaypoint],
    step_size: float = 0.005,
) -> Tuple[List[dict], dict]:
    """Convert waypoints to animation states with erasure tracking."""
    sim_states = []
    ink_visible = None
    total_erased = 0.0
    total_drawn = 0.0
    
    for i in range(len(waypoints)):
        wp = waypoints[i]
        
        if i > 0:
            prev_wp = waypoints[i - 1]
            dx = wp.x - prev_wp.x
            dy = wp.y - prev_wp.y
            dist = math.hypot(dx, dy)
            n_steps = max(1, int(dist / step_size))
            
            for step in range(1, n_steps + 1):
                t = step / n_steps
                interp_x = prev_wp.x + t * dx
                interp_y = prev_wp.y + t * dy
                dtheta = normalize_angle(wp.theta - prev_wp.theta)
                interp_theta = normalize_angle(prev_wp.theta + t * dtheta)
                
                robot_poly = get_robot_body(interp_x, interp_y, interp_theta)
                
                # Erase ink touched by robot
                if ink_visible is not None:
                    length_before = line_length(ink_visible)
                    try:
                        diff = ink_visible.difference(robot_poly)
                        ink_visible = diff if diff is not None and not diff.is_empty else None
                    except Exception:
                        pass
                    length_after = line_length(ink_visible) if ink_visible is not None else 0.0
                    total_erased += max(0, length_before - length_after)
                
                # Draw ink if pen is down
                if prev_wp.pen_down and wp.pen_down:
                    prev_t = (step - 1) / n_steps
                    prev_x = prev_wp.x + prev_t * dx
                    prev_y = prev_wp.y + prev_t * dy
                    segment = LineString([(prev_x, prev_y), (interp_x, interp_y)])
                    if segment.length > 1e-6:
                        total_drawn += segment.length
                        ink_visible = segment if ink_visible is None else unary_union([ink_visible, segment])
                
                sim_states.append({"robot_poly": robot_poly, "ink": ink_visible})
        else:
            robot_poly = get_robot_body(wp.x, wp.y, wp.theta)
            sim_states.append({"robot_poly": robot_poly, "ink": ink_visible})
    
    return sim_states, {
        "total_erased": total_erased,
        "total_drawn": total_drawn,
        "erasure_ratio": total_erased / total_drawn if total_drawn > 0 else 0.0,
    }


def animate(target_geom, sim_states, metrics, canvas_height):
    """Run the drawing animation."""
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.7)

    if target_geom is not None and not target_geom.is_empty:
        ref_segments = geometry_to_segments(target_geom)
        ref_coll = LineCollection(ref_segments, colors="gray", linewidths=1.0, linestyles="dashed", label="Target")
        ax.add_collection(ref_coll)

    robot_patch = ax.fill([], [], alpha=0.7, fc="green", ec="black", label=f"Robot")[0]
    ink_collection = LineCollection([], colors="red", linewidths=2.5, label="Visible Ink")
    ax.add_collection(ink_collection)

    ax.set_xlim(-CANVAS_PADDING - 0.2, TARGET_CANVAS_WIDTH + CANVAS_PADDING)
    ax.set_ylim(-CANVAS_PADDING - 0.2, canvas_height + CANVAS_PADDING)
    legend = ax.legend(loc="upper right")

    def update(frame_idx):
        st = sim_states[frame_idx]
        poly = st["robot_poly"]
        x_poly, y_poly = poly.exterior.xy
        robot_patch.set_xy(list(zip(x_poly, y_poly)))
        segments = geometry_to_segments(st["ink"])
        ink_collection.set_segments(segments)
        ax.set_title(
            f"Frame {frame_idx+1}/{len(sim_states)} | "
            f"Drawn: {metrics['drawn_length']:.2f}m | Erased: {metrics['erased_length']:.2f}m"
        )
        return robot_patch, ink_collection, legend

    ani = animation.FuncAnimation(
        fig, update, frames=len(sim_states),
        interval=ANIMATION_INTERVAL_MS, blit=True, repeat=False,
    )
    ACTIVE_ANIMATIONS.append(ani)
    plt.show()


def visualize_polylines(polylines, title="Polylines"):
    """Simple polyline visualization."""
    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    for poly in polylines:
        if len(poly) >= 2:
            xs, ys = zip(*poly)
            ax.plot(xs, ys, linewidth=1.5)
    ax.set_title(title)
    ax.grid(True, linestyle=":", alpha=0.5)
    plt.show()


def run(args, image_path: str):
    """Main entry point for processing an image."""
    stages = build_image_stages(image_path, args.dedup_tol)

    if args.view == "image":
        visualize_image_stages(stages)
        return

    if args.view == "polylines":
        visualize_polylines(stages.rescaled_polylines, title="Extracted Polylines")
        return

    run_pipeline(args, stages)


def parse_args():
    parser = argparse.ArgumentParser(description="Drawing robot path planner")
    parser.add_argument("image_path", nargs="?", help="Path to line art image")
    parser.add_argument(
        "--view",
        choices=["image", "polylines", "full"],
        default="full",
        help="What to visualize (default: full simulation)",
    )
    parser.add_argument("--no-animate", action="store_true", help="Skip animation")
    parser.add_argument(
        "--samples-per-stroke",
        type=int,
        default=100,
        help="Waypoints per stroke (default: 100)",
    )
    parser.add_argument(
        "--dedup-tol",
        type=float,
        default=DEDUPLICATION_DISTANCE_TOLERANCE_DEFAULT,
        help="Tolerance for merging duplicate polylines",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    
    image_path = args.image_path
    if not image_path:
        image_path = input("Image path: ").strip()
    if not image_path:
        print("No image provided.")
        return
    
    run(args, image_path)


if __name__ == "__main__":
    print("=" * 60)
    print("HOLONOMIC DRAWING ROBOT PATH PLANNER")
    print("=" * 60)
    print(f"Robot: Triangle, Side={ROBOT_SIDE_LENGTH:.4f}m")
    print(f"Marker offset: ({MARKER_OFFSET_X:.4f}, {MARKER_OFFSET_Y:.4f})")
    print(f"Canvas: {TARGET_CANVAS_WIDTH}m wide")
    print(f"Dock: {DOCK_POSITION} (approach: {DOCK_APPROACH})")
    print("-" * 60)
    main()

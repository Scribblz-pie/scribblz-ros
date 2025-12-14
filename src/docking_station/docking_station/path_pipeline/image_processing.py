"""Image processing utilities for the drawing robot pipeline.

Converts images to polylines using cv2 and skeletonization.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import json
import math
import os

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from shapely.geometry import LineString
from shapely.ops import unary_union

import networkx as nx

# Required dependencies
import cv2
from skimage import io as skio
from skimage.color import rgb2gray
from skimage.filters import threshold_otsu
from skimage.morphology import skeletonize

DEDUPLICATION_DISTANCE_TOLERANCE_DEFAULT = 0.005  # Canvas units (meters)


@dataclass
class ImageStages:
    """Container holding intermediate outputs of the image-to-path pipeline."""
    image: np.ndarray
    mask: np.ndarray
    skeleton: np.ndarray
    polylines: List[List[Tuple[float, float]]]
    hierarchy: Optional[np.ndarray]
    rescaled_polylines: List[List[Tuple[float, float]]]
    bounds: Tuple[float, float, float, float]
    canvas_height: float


@dataclass
class RobotPose:
    """Robot pose in canvas coordinates."""
    x: float
    y: float
    yaw: float


@dataclass
class BodyTwist:
    """Body-frame velocity command."""
    vx: float
    vy: float
    omega: float
    pen_down: bool = False


@dataclass
class WheelCommand:
    """Timed wheel command entry suitable for simulation or playback."""
    timestamp: float
    duration: float
    pose: RobotPose
    body_twist: BodyTwist
    world_velocity: Tuple[float, float]
    wheel_speeds: Tuple[float, float, float]
    pen_down: bool = False
    polyline_index: Optional[int] = None
    segment_index: Optional[int] = None


def _world_to_body_velocity(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
    """Transform world velocity to body frame."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    bx = cos_yaw * vx + sin_yaw * vy
    by = -sin_yaw * vx + cos_yaw * vy
    return bx, by


def compute_kiwi_wheel_speeds(
    vx_body: float,
    vy_body: float,
    omega: float,
    robot_radius: float,
) -> np.ndarray:
    """Compute wheel speeds using inverse kinematics for Kiwi drive."""
    if robot_radius <= 0:
        raise ValueError("robot_radius must be positive")

    l = robot_radius
    sqrt3 = np.sqrt(3.0)

    w1 = (vy_body + l * omega)
    w2 = (sqrt3 / 2.0 * vx_body - 0.5 * vy_body + l * omega)
    w3 = (-sqrt3 / 2.0 * vx_body - 0.5 * vy_body + l * omega)
    
    return np.array([w1, w2, w3], dtype=float)


def _poly_to_linestring(poly: Sequence[Tuple[float, float]]) -> Optional[LineString]:
    """Convert polyline to Shapely LineString."""
    try:
        return LineString(poly)
    except Exception:
        clean = [poly[0]]
        for p in poly[1:]:
            if p != clean[-1]:
                clean.append(p)
        if len(clean) >= 2:
            return LineString(clean)
        return None


def save_commands_to_json(
    commands: List[WheelCommand],
    filepath: str,
    metadata: dict,
    wheel_radius: Optional[float] = None,
    indent: int = 2,
) -> None:
    """Export wheel commands to JSON with waypoints format."""
    waypoints = []
    for cmd in commands:
        waypoint = {
            "x": cmd.pose.x,
            "y": cmd.pose.y,
            "yaw": cmd.pose.yaw,
            "pen_down": cmd.pen_down,
            "timestamp": cmd.timestamp,
            "duration": cmd.duration,
        }
        waypoints.append(waypoint)
        
    output_data = {
        "metadata": metadata.copy(),
        "waypoints": waypoints,
    }
    
    with open(filepath, "w", encoding="utf-8") as handle:
        json.dump(output_data, handle, indent=indent)


def load_image(image_path: str) -> np.ndarray:
    """Load an image and normalize to [0, 1]."""
    if not os.path.isfile(image_path):
        raise FileNotFoundError(f"Image not found: {image_path}")

    img = skio.imread(image_path).astype(float)

    if img.ndim == 3 and img.shape[-1] == 4:
        img = img[..., :3]  # Drop alpha channel

    if img.max() > 1.0:
        img = img / img.max()

    return img


def binarize_image(img: np.ndarray) -> np.ndarray:
    """Return boolean mask where True = ink pixels."""
    if img.ndim == 3:
        gray = rgb2gray(img)
    else:
        gray = img.astype(float)
        gray = gray / (gray.max() if gray.max() else 1.0)

    t = threshold_otsu(gray)
    return gray < t


def skeleton_to_polylines(skeleton_mask: np.ndarray) -> List[List[Tuple[float, float]]]:
    """Trace a 1-pixel wide skeleton into polylines using graph traversal."""
    pixels = np.argwhere(skeleton_mask > 0)
    if len(pixels) == 0:
        return []

    h, w = skeleton_mask.shape
    G = nx.Graph()
    
    nodes = [(int(y), int(x)) for y, x in pixels]
    node_set = set(nodes)
    G.add_nodes_from(nodes)

    # Add 8-connected edges
    for r, c in nodes:
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if (nr, nc) in node_set:
                    dist = math.hypot(dr, dc)
                    G.add_edge((r, c), (nr, nc), weight=dist)

    polylines: List[List[Tuple[float, float]]] = []
    
    for component in nx.connected_components(G):
        subG = G.subgraph(component).copy()
        
        if subG.number_of_nodes() < 2:
            continue

        # Find terminals (endpoints or junctions)
        terminals = [n for n, d in subG.degree() if d != 2]
        if not terminals:
            terminals = [list(subG.nodes())[0]]
        
        visited_edges = set()
        
        def path_to_poly(p):
            return [(float(c), float(h - 1 - r)) for r, c in p]

        for start_node in terminals:
            for neighbor in subG.neighbors(start_node):
                edge_key = tuple(sorted((start_node, neighbor)))
                if edge_key in visited_edges:
                    continue
                
                path = [start_node, neighbor]
                visited_edges.add(edge_key)
                
                curr = neighbor
                while True:
                    if subG.degree(curr) != 2:
                        if curr != start_node or len(path) > 2:
                            break
                    
                    found_next = False
                    for nbr in subG.neighbors(curr):
                        e_key = tuple(sorted((curr, nbr)))
                        if e_key not in visited_edges:
                            visited_edges.add(e_key)
                            path.append(nbr)
                            curr = nbr
                            found_next = True
                            break
                    
                    if not found_next:
                        break
                
                if len(path) >= 2:
                    polylines.append(path_to_poly(path))

    return polylines


def dedupe_empty_polylines(polylines: Sequence[Sequence[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:
    """Remove polylines with fewer than 2 points."""
    return [list(p) for p in polylines if len(p) >= 2]


def _canonical_poly_key(poly: Sequence[Tuple[float, float]], precision: int = 5):
    """Create canonical key for polyline deduplication."""
    if len(poly) < 2:
        return None
    rounded = tuple((round(x, precision), round(y, precision)) for x, y in poly)
    reversed_rounded = tuple(reversed(rounded))
    return rounded if rounded <= reversed_rounded else reversed_rounded


def deduplicate_polylines(
    polylines: Sequence[Sequence[Tuple[float, float]]],
    tolerance: float = DEDUPLICATION_DISTANCE_TOLERANCE_DEFAULT,
) -> List[List[Tuple[float, float]]]:
    """Remove duplicate and near-duplicate polylines."""
    cleaned: List[List[Tuple[float, float]]] = []
    seen_keys: set = set()

    for poly in polylines:
        key = _canonical_poly_key(poly)
        if key is None or key in seen_keys:
            continue
        seen_keys.add(key)
        cleaned.append(list(poly))

    if tolerance <= 0:
        return cleaned

    unique_lines: List[List[Tuple[float, float]]] = []
    line_geoms: List[LineString] = []
    
    for poly in cleaned:
        ls = _poly_to_linestring(poly)
        if ls is None or ls.is_empty:
            continue

        is_duplicate = False
        for existing in line_geoms:
            try:
                if ls.distance(existing) < tolerance:
                    is_duplicate = True
                    break
            except Exception:
                continue

        if not is_duplicate:
            unique_lines.append(list(poly))
            line_geoms.append(ls)

    return unique_lines


def prune_micro_segments(
    polylines: Sequence[Sequence[Tuple[float, float]]],
    min_len: float = 0.25
) -> List[List[Tuple[float, float]]]:
    """Remove extremely short segments from polylines."""
    cleaned: List[List[Tuple[float, float]]] = []
    for poly in polylines:
        if len(poly) < 2:
            continue
        new_poly: List[Tuple[float, float]] = [poly[0]]
        for i in range(1, len(poly)):
            x0, y0 = new_poly[-1]
            x1, y1 = poly[i]
            if np.hypot(x1 - x0, y1 - y0) >= min_len:
                new_poly.append((x1, y1))
        if len(new_poly) >= 2:
            cleaned.append(new_poly)
    return cleaned


def rescale_polylines_to_canvas(
    polylines: Sequence[Sequence[Tuple[float, float]]],
    target_width: float,
    padding: float,
) -> Tuple[List[List[Tuple[float, float]]], Tuple[float, float, float, float], float]:
    """Rescale polylines to fit within target canvas dimensions."""
    if not polylines or all(len(p) == 0 for p in polylines):
        return dedupe_empty_polylines(polylines), (0, 0, target_width, target_width), target_width

    all_lines = [_poly_to_linestring(p) for p in polylines if _poly_to_linestring(p)]
    if not all_lines:
        return dedupe_empty_polylines(polylines), (0, 0, target_width, target_width), target_width

    all_geom = unary_union(all_lines)
    minx, miny, maxx, maxy = all_geom.bounds
    data_w = maxx - minx
    data_h = maxy - miny

    if data_w < 1e-6 or data_h < 1e-6:
        scale = 1.0
        target_height = target_width
    else:
        target_height = target_width * (data_h / data_w)
        scale = min(
            (target_width - 2 * padding) / data_w,
            (target_height - 2 * padding) / data_h,
        )

    new_w, new_h = data_w * scale, data_h * scale
    offset_x = padding + (target_width - 2 * padding - new_w) / 2.0 - (minx * scale)
    offset_y = padding + (target_height - 2 * padding - new_h) / 2.0 - (miny * scale)

    grid = max(1e-6, min((target_width - 2 * padding), (target_height - 2 * padding)) / 500.0)

    new_polylines: List[List[Tuple[float, float]]] = []
    for p in polylines:
        new_p = []
        for x, y in p:
            nx = round((x * scale + offset_x) / grid) * grid
            ny = round((y * scale + offset_y) / grid) * grid
            new_p.append((nx, ny))
        new_polylines.append(new_p)

    bounds = (minx * scale + offset_x, miny * scale + offset_y, 
              maxx * scale + offset_x, maxy * scale + offset_y)
    return new_polylines, bounds, target_height


def generate_image_stages(
    image_path: str,
    target_width: float,
    padding: float,
    smooth_factor: float = 0.0,
    smoothing_points: int = 200,
    simplification_epsilon_factor: float = 0.0,
    dedup_tolerance: float = DEDUPLICATION_DISTANCE_TOLERANCE_DEFAULT,
    extractor: str = "cv2",  # Ignored, always uses cv2+skeleton
    approx_tol: float = 0.0,  # Ignored
    use_skeleton: bool = True,  # Ignored, always True
) -> ImageStages:
    """Process image to extract polylines.
    
    Always uses cv2 + skeletonization for best results.
    """
    image = load_image(image_path)
    mask = binarize_image(image)
    
    # Always skeletonize
    skeleton = skeletonize(mask)
    polylines = skeleton_to_polylines(skeleton)
    
    polylines = dedupe_empty_polylines(polylines)
    polylines = deduplicate_polylines(polylines, tolerance=0.0)

    rescaled_polys, bounds, canvas_height = rescale_polylines_to_canvas(
        polylines, target_width, padding
    )
    rescaled_polys = prune_micro_segments(rescaled_polys, min_len=max(0.01, target_width * 0.002))
    deduped_rescaled = deduplicate_polylines(rescaled_polys, tolerance=dedup_tolerance)

    return ImageStages(
        image=image,
        mask=mask,
        skeleton=skeleton.astype(np.uint8) * 255,
        polylines=polylines,
        hierarchy=None,
        rescaled_polylines=deduped_rescaled,
        bounds=bounds,
        canvas_height=canvas_height,
    )


def visualize_image_stages(stages: ImageStages, show: bool = True):
    """Visualize the image-to-path pipeline stages."""
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    ax_orig, ax_mask, ax_skel = axes[0]
    ax_poly, ax_rescaled, ax_info = axes[1]

    ax_orig.imshow(stages.image, cmap="gray")
    ax_orig.set_title("Original Image")
    ax_orig.axis("off")

    ax_mask.imshow(stages.mask, cmap="gray")
    ax_mask.set_title("Binarized Mask")
    ax_mask.axis("off")

    ax_skel.imshow(stages.skeleton, cmap="gray")
    ax_skel.set_title("Skeleton")
    ax_skel.axis("off")

    def plot_polylines(ax, polys, title):
        ax.set_aspect("equal")
        ax.set_title(title)
        ax.grid(True, linestyle=":", alpha=0.3)
        segments = []
        for poly in polys:
            if len(poly) >= 2:
                for i in range(len(poly) - 1):
                    segments.append([poly[i], poly[i + 1]])
        if segments:
            coll = LineCollection(segments, colors="red", linewidths=1.5)
            ax.add_collection(coll)
        ax.autoscale()

    plot_polylines(ax_poly, stages.polylines, f"Polylines ({len(stages.polylines)})")
    plot_polylines(ax_rescaled, stages.rescaled_polylines, 
                   f"Rescaled ({len(stages.rescaled_polylines)})")

    ax_info.axis("off")
    ax_info.text(0.5, 0.5, 
                 f"Bounds: {stages.bounds}\n"
                 f"Canvas height: {stages.canvas_height:.2f}m\n"
                 f"Polylines: {len(stages.rescaled_polylines)}",
                 ha="center", va="center", fontsize=12)

    plt.tight_layout()
    if show:
        plt.show()


__all__ = [
    "ImageStages",
    "generate_image_stages",
    "visualize_image_stages",
    "RobotPose",
    "BodyTwist",
    "WheelCommand",
    "compute_kiwi_wheel_speeds",
    "save_commands_to_json",
    "_world_to_body_velocity",
    "_poly_to_linestring",
    "DEDUPLICATION_DISTANCE_TOLERANCE_DEFAULT",
]

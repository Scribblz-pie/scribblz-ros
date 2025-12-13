"""Stroke ordering optimization using TSP-based algorithms.

This module provides global optimization for stroke ordering to minimize
pen-up travel distance while respecting containment constraints (inside-out).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple, Set
import math

import numpy as np

from shapely.geometry import LineString, MultiPoint, Polygon
from shapely.ops import unary_union


@dataclass
class Stroke:
    """A drawable stroke with endpoints and geometry."""
    
    index: int  # Original index in the polylines list
    start: Tuple[float, float]
    end: Tuple[float, float]
    geometry: LineString
    arc_length: float
    is_closed: bool
    
    def reversed(self) -> "Stroke":
        """Return a copy with start/end swapped."""
        return Stroke(
            index=self.index,
            start=self.end,
            end=self.start,
            geometry=LineString(list(self.geometry.coords)[::-1]),
            arc_length=self.arc_length,
            is_closed=self.is_closed,
        )
    
    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """Return (minx, miny, maxx, maxy)."""
        return self.geometry.bounds
    
    @property
    def centroid(self) -> Tuple[float, float]:
        """Return centroid of the stroke."""
        c = self.geometry.centroid
        return (c.x, c.y)


@dataclass
class OrderedStroke:
    """A stroke with its drawing direction determined."""
    
    stroke: Stroke
    draw_reversed: bool  # If True, draw from end to start
    
    @property
    def draw_start(self) -> Tuple[float, float]:
        return self.stroke.end if self.draw_reversed else self.stroke.start
    
    @property
    def draw_end(self) -> Tuple[float, float]:
        return self.stroke.start if self.draw_reversed else self.stroke.end
    
    @property
    def ordered_geometry(self) -> LineString:
        if self.draw_reversed:
            return LineString(list(self.stroke.geometry.coords)[::-1])
        return self.stroke.geometry


def strokes_from_polylines(
    polylines: Sequence[Sequence[Tuple[float, float]]],
) -> List[Stroke]:
    """Convert polylines to Stroke objects."""
    strokes = []
    for i, poly in enumerate(polylines):
        if len(poly) < 2:
            continue
        
        coords = list(poly)
        try:
            ls = LineString(coords)
            if ls.is_empty or ls.length < 1e-6:
                continue
        except Exception:
            continue
        
        start = (float(coords[0][0]), float(coords[0][1]))
        end = (float(coords[-1][0]), float(coords[-1][1]))
        is_closed = math.hypot(end[0] - start[0], end[1] - start[1]) < ls.length * 0.02
        
        strokes.append(Stroke(
            index=i,
            start=start,
            end=end,
            geometry=ls,
            arc_length=ls.length,
            is_closed=is_closed,
        ))
    
    return strokes


def _distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Euclidean distance between two points."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def build_distance_matrix(strokes: List[Stroke]) -> np.ndarray:
    """Build distance matrix for TSP.
    
    For N strokes, creates a 2N x 2N matrix where:
    - Node 2i represents stroke i drawn forward (start -> end)
    - Node 2i+1 represents stroke i drawn backward (end -> start)
    
    Distance from node A_end to node B_start is the pen-up travel distance.
    """
    n = len(strokes)
    if n == 0:
        return np.array([[]])
    
    # 2N nodes: each stroke has 2 representations (forward/backward)
    size = 2 * n
    dist = np.full((size, size), np.inf)
    
    for i in range(n):
        for j in range(n):
            if i == j:
                # Can't go from a stroke to itself
                continue
            
            # Stroke i forward ends at strokes[i].end
            # Stroke i backward ends at strokes[i].start
            i_fwd_end = strokes[i].end
            i_bwd_end = strokes[i].start
            
            # Stroke j forward starts at strokes[j].start
            # Stroke j backward starts at strokes[j].end
            j_fwd_start = strokes[j].start
            j_bwd_start = strokes[j].end
            
            # i forward -> j forward
            dist[2*i, 2*j] = _distance(i_fwd_end, j_fwd_start)
            # i forward -> j backward
            dist[2*i, 2*j+1] = _distance(i_fwd_end, j_bwd_start)
            # i backward -> j forward
            dist[2*i+1, 2*j] = _distance(i_bwd_end, j_fwd_start)
            # i backward -> j backward
            dist[2*i+1, 2*j+1] = _distance(i_bwd_end, j_bwd_start)
    
    return dist


def _greedy_tsp(dist: np.ndarray, start_node: int = 0) -> List[int]:
    """Greedy nearest-neighbor TSP heuristic.
    
    Returns list of node indices in visit order.
    """
    n = dist.shape[0]
    if n == 0:
        return []
    
    visited = {start_node}
    tour = [start_node]
    current = start_node
    
    while len(visited) < n:
        # Find nearest unvisited node
        min_dist = np.inf
        next_node = -1
        
        for j in range(n):
            if j not in visited and dist[current, j] < min_dist:
                min_dist = dist[current, j]
                next_node = j
        
        if next_node == -1:
            # No reachable unvisited nodes, find any unvisited
            for j in range(n):
                if j not in visited:
                    next_node = j
                    break
        
        if next_node == -1:
            break
        
        visited.add(next_node)
        tour.append(next_node)
        current = next_node
    
    return tour


def _two_opt(tour: List[int], dist: np.ndarray, max_iterations: int = 1000) -> List[int]:
    """2-opt local search to improve TSP tour.
    
    Repeatedly reverses segments that reduce total distance.
    """
    improved = True
    iterations = 0
    tour = list(tour)
    n = len(tour)
    
    if n < 4:
        return tour
    
    def tour_distance(t: List[int]) -> float:
        total = 0.0
        for i in range(len(t) - 1):
            d = dist[t[i], t[i+1]]
            if d == np.inf:
                d = 1e9  # Large but finite
            total += d
        return total
    
    while improved and iterations < max_iterations:
        improved = False
        iterations += 1
        
        for i in range(n - 2):
            for j in range(i + 2, n):
                # Current edges: (i, i+1) and (j, j+1 or wrap)
                # After reversal: (i, j) and (i+1, j+1)
                
                if j == n - 1:
                    # Edge case at the end
                    continue
                
                # Calculate distance change
                d1 = dist[tour[i], tour[i+1]]
                d2 = dist[tour[j], tour[j+1]] if j+1 < n else 0
                d3 = dist[tour[i], tour[j]]
                d4 = dist[tour[i+1], tour[j+1]] if j+1 < n else 0
                
                # Handle infinite distances
                if d1 == np.inf:
                    d1 = 1e9
                if d2 == np.inf:
                    d2 = 1e9
                if d3 == np.inf:
                    d3 = 1e9
                if d4 == np.inf:
                    d4 = 1e9
                
                if d3 + d4 < d1 + d2 - 1e-9:
                    # Reverse segment between i+1 and j
                    tour[i+1:j+1] = reversed(tour[i+1:j+1])
                    improved = True
    
    return tour


def _extract_stroke_order(tour: List[int], n_strokes: int) -> List[OrderedStroke]:
    """Convert TSP tour (over 2N nodes) to ordered strokes.
    
    Handles the constraint that each stroke is visited exactly once
    (either forward or backward variant).
    """
    visited_strokes: Set[int] = set()
    ordered: List[Tuple[int, bool]] = []  # (stroke_index, reversed)
    
    for node in tour:
        stroke_idx = node // 2
        is_reversed = (node % 2 == 1)
        
        if stroke_idx not in visited_strokes:
            visited_strokes.add(stroke_idx)
            ordered.append((stroke_idx, is_reversed))
    
    return ordered


def solve_tsp_ordering(
    strokes: List[Stroke],
    start_position: Optional[Tuple[float, float]] = None,
) -> List[OrderedStroke]:
    """Solve TSP to find optimal stroke ordering.
    
    Args:
        strokes: List of strokes to order
        start_position: Optional starting position (e.g., (0, 0))
    
    Returns:
        List of OrderedStroke with optimal drawing order and direction
    """
    if len(strokes) == 0:
        return []
    
    if len(strokes) == 1:
        return [OrderedStroke(strokes[0], draw_reversed=False)]
    
    # Build distance matrix
    dist = build_distance_matrix(strokes)
    
    # Find best starting node
    if start_position is not None:
        # Find node closest to start_position
        min_dist = np.inf
        start_node = 0
        for i, stroke in enumerate(strokes):
            d_fwd = _distance(start_position, stroke.start)
            d_bwd = _distance(start_position, stroke.end)
            if d_fwd < min_dist:
                min_dist = d_fwd
                start_node = 2 * i
            if d_bwd < min_dist:
                min_dist = d_bwd
                start_node = 2 * i + 1
    else:
        start_node = 0
    
    # Greedy initial tour
    tour = _greedy_tsp(dist, start_node)
    
    # Improve with 2-opt
    tour = _two_opt(tour, dist)
    
    # Extract stroke order
    stroke_order = _extract_stroke_order(tour, len(strokes))
    
    # Build result
    result = []
    for stroke_idx, is_reversed in stroke_order:
        result.append(OrderedStroke(strokes[stroke_idx], draw_reversed=is_reversed))
    
    return result


def _get_stroke_hull(stroke: Stroke) -> Polygon:
    """Get convex hull of stroke for containment checking."""
    try:
        hull = stroke.geometry.convex_hull
        if isinstance(hull, Polygon):
            return hull
        # For lines, buffer slightly to create a polygon
        return stroke.geometry.buffer(0.001)
    except Exception:
        return Polygon()


def build_containment_graph(strokes: List[Stroke]) -> List[Set[int]]:
    """Build containment relationships between strokes.
    
    Returns:
        List where contains[i] is the set of stroke indices contained by stroke i
    """
    n = len(strokes)
    contains: List[Set[int]] = [set() for _ in range(n)]
    
    hulls = [_get_stroke_hull(s) for s in strokes]
    
    for i in range(n):
        hull_i = hulls[i]
        if hull_i.is_empty:
            continue
        
        for j in range(n):
            if i == j:
                continue
            
            hull_j = hulls[j]
            if hull_j.is_empty:
                continue
            
            try:
                # Check if i contains j
                if hull_i.contains(hull_j) or hull_i.buffer(0.001).contains(hull_j):
                    contains[i].add(j)
            except Exception:
                continue
    
    return contains


def _topological_sort_inside_out(
    strokes: List[Stroke],
    contains: List[Set[int]],
) -> List[int]:
    """Sort strokes so inner strokes are drawn before outer strokes."""
    n = len(strokes)
    
    # Build dependency graph: if i contains j, then j must be drawn before i
    # So j -> i in dependency graph
    in_degree = [0] * n
    dependents: List[Set[int]] = [set() for _ in range(n)]
    
    for i in range(n):
        for j in contains[i]:
            # j is inside i, so j must be drawn first
            # j -> i means i depends on j
            dependents[j].add(i)
            in_degree[i] += 1
    
    # Kahn's algorithm for topological sort
    queue = [i for i in range(n) if in_degree[i] == 0]
    result = []
    
    while queue:
        # Among available nodes, pick one (could optimize by proximity)
        node = queue.pop(0)
        result.append(node)
        
        for dep in dependents[node]:
            in_degree[dep] -= 1
            if in_degree[dep] == 0:
                queue.append(dep)
    
    # Handle any remaining nodes (cycles)
    for i in range(n):
        if i not in result:
            result.append(i)
    
    return result


def order_strokes_with_containment(
    strokes: List[Stroke],
    start_position: Optional[Tuple[float, float]] = None,
) -> List[OrderedStroke]:
    """Order strokes respecting containment (inside-out) and minimizing travel.
    
    This combines containment-based ordering with TSP optimization within
    each containment level.
    """
    if len(strokes) == 0:
        return []
    
    if len(strokes) == 1:
        return [OrderedStroke(strokes[0], draw_reversed=False)]
    
    # Build containment graph
    contains = build_containment_graph(strokes)
    
    # Get inside-out order
    topo_order = _topological_sort_inside_out(strokes, contains)
    
    # Reorder strokes according to topological order
    ordered_strokes = [strokes[i] for i in topo_order]
    
    # Now optimize within this order using TSP-like approach
    # But we must maintain the inside-out constraint
    
    # Group strokes by containment depth
    depth = [0] * len(strokes)
    for i in range(len(strokes)):
        # Depth is how many strokes contain this one
        for j in range(len(strokes)):
            if i in contains[j]:
                depth[i] += 1
    
    # Group by depth
    depth_groups: dict[int, List[int]] = {}
    for i, d in enumerate(depth):
        if d not in depth_groups:
            depth_groups[d] = []
        depth_groups[d].append(i)
    
    # Process each depth level, optimizing order within level
    result: List[OrderedStroke] = []
    current_pos = start_position or (0.0, 0.0)
    
    for d in sorted(depth_groups.keys(), reverse=True):  # Deepest first (inside-out)
        level_strokes = [strokes[i] for i in depth_groups[d]]
        
        if len(level_strokes) == 0:
            continue
        
        # TSP within this level
        level_ordered = solve_tsp_ordering(level_strokes, start_position=current_pos)
        
        for os in level_ordered:
            result.append(os)
            current_pos = os.draw_end
    
    return result


def compute_total_travel(ordered_strokes: List[OrderedStroke]) -> float:
    """Compute total pen-up travel distance for an ordering."""
    if len(ordered_strokes) < 2:
        return 0.0
    
    total = 0.0
    for i in range(len(ordered_strokes) - 1):
        end = ordered_strokes[i].draw_end
        start = ordered_strokes[i + 1].draw_start
        total += _distance(end, start)
    
    return total


__all__ = [
    "Stroke",
    "OrderedStroke",
    "strokes_from_polylines",
    "solve_tsp_ordering",
    "order_strokes_with_containment",
    "compute_total_travel",
]


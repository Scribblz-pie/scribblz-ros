#!/usr/bin/env python3

"""
Circle fitting and coordinate calculation for lidar pose estimation.

This module handles:
- Converting lidar scan data to Cartesian coordinates
- Filtering valid points by distance
- Clustering points
- Fitting circles using RANSAC algorithm
- Calculating robot pose from detected circles
"""

import math
import numpy as np


class CircleFitter:
    """
    Fits circles to lidar scan data to detect docking station position.
    
    Uses RANSAC algorithm to robustly detect circular targets in the scan.
    """
    
    def find_small_clusters(self, points, min_pts, max_pts, dist_thresh):
        """
        Find clusters of points within a distance threshold.
        Optimized with vectorized operations and boolean masks.
        
        Args:
            points: numpy array of (x, y) points
            min_pts: Minimum points required in a cluster
            max_pts: Maximum points allowed in a cluster
            dist_thresh: Distance threshold for clustering (cm)
            
        Returns:
            List of numpy arrays, each containing points in a cluster
        """
        if len(points) < min_pts:
            return []

        n_points = len(points)
        unvisited = np.ones(n_points, dtype=bool)
        clusters = []

        while np.any(unvisited):
            # Get first unvisited point as seed
            seed = np.where(unvisited)[0][0]
            unvisited[seed] = False
            cluster_indices = [seed]
            to_check = [seed]

            while to_check:
                current = to_check.pop()
                current_point = points[current]

                # Vectorized distance computation for all unvisited points
                if np.any(unvisited):
                    unvisited_indices = np.where(unvisited)[0]
                    if len(unvisited_indices) > 0:
                        unvisited_points = points[unvisited_indices]
                        # Optimized distance calculation
                        diff = unvisited_points - current_point
                        distances = np.sqrt(np.sum(diff * diff, axis=1))
                        nearby_mask = distances < dist_thresh

                        # Process all nearby points at once
                        nearby_indices = unvisited_indices[nearby_mask]
                        for idx in nearby_indices:
                            unvisited[idx] = False
                            cluster_indices.append(idx)
                            to_check.append(idx)

            if min_pts <= len(cluster_indices) <= max_pts:
                clusters.append(points[cluster_indices])

        clusters.sort(key=len)
        return clusters

    def filter_valid_points(self, points, min_dist, max_dist):
        """
        Filter points by distance from origin.
        Optimized with vectorized squared distances to avoid sqrt when possible.
        
        Args:
            points: numpy array of (x, y) points (cm)
            min_dist: Minimum distance from origin (cm)
            max_dist: Maximum distance from origin (cm)
            
        Returns:
            Filtered numpy array of points
        """
        # Use squared distances for faster comparison
        distances_sq = points[:, 0] ** 2 + points[:, 1] ** 2
        valid_mask = (distances_sq > min_dist ** 2) & (distances_sq < max_dist ** 2)
        return points[valid_mask]

    def _fit_circle_algebraic(self, points):
        """
        Fit a circle to points using algebraic least squares.
        Optimized with vectorized operations and efficient matrix construction.
        
        Args:
            points: numpy array of at least 3 (x, y) points
            
        Returns:
            Tuple of (center_x, center_y, radius) in cm, or None if fit fails
        """
        if len(points) < 3:
            return None
        try:
            x = points[:, 0]
            y = points[:, 1]
            # Optimized matrix construction
            n = len(points)
            A = np.empty((n, 3))
            A[:, 0] = 2 * x
            A[:, 1] = 2 * y
            A[:, 2] = 1.0
            # Pre-compute x² + y² (cached for reuse)
            b = x * x + y * y
            # Solve using lstsq (internally uses optimized LAPACK)
            params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            cx, cy, D = params
            r_squared = cx * cx + cy * cy + D
            if r_squared <= 0:
                return None
            r = np.sqrt(r_squared)  # Use numpy sqrt for consistency
            return (cx, cy, r)
        except Exception:
            return None

    def fit_circle_ransac(self, angles, distances, cfg):
        """
        Fit a circle to lidar scan data using RANSAC algorithm.
        
        Args:
            angles: List of angles in degrees
            distances: List of distances in meters
            cfg: Configuration dict with keys:
                - min_circle_points: Minimum points for valid circle
                - expected_radius_cm: Expected circle radius (cm)
                - radius_tolerance_cm: Allowed radius deviation (cm)
                - ransac_iterations: Number of RANSAC iterations
                - inlier_threshold_cm: Distance threshold for inliers (cm)
                - min_detection_distance_cm: Minimum detection distance (cm)
                - max_detection_distance_cm: Maximum detection distance (cm)
                
        Returns:
            Tuple of (center_x_cm, center_y_cm, radius_cm, inlier_count) or None
        """
        if len(angles) < cfg["min_circle_points"]:
            return None

        # Convert polar to Cartesian coordinates (in cm) - VECTORIZED
        # Lidar frame convention: 
        # - x axis: forward (0° = +x direction, standard ROS convention)
        # - y axis: left (90° = +y direction)
        # - z axis: up
        # The static transform from lidar to base_link handles the actual mounting orientation
        angles_np = np.array(angles)
        distances_np = np.array(distances)
        angles_rad = np.radians(angles_np)
        points = np.column_stack([
            distances_np * 100.0 * np.cos(angles_rad),
            distances_np * 100.0 * np.sin(angles_rad)
        ])

        # Filter by distance
        valid_points = self.filter_valid_points(
            points, 
            cfg["min_detection_distance_cm"], 
            cfg["max_detection_distance_cm"]
        )
        if len(valid_points) < cfg["min_circle_points"]:
            return None

        # Find clusters
        clusters = self.find_small_clusters(
            valid_points, 
            cfg["min_circle_points"], 
            30,  # max_pts
            10.0  # dist_thresh (cm)
        )
        if len(clusters) == 0:
            return None

        # RANSAC fitting with early termination
        best_fit = None
        best_score = float('inf')
        early_termination_threshold = 0.5  # Stop if score is excellent

        for cluster_points in clusters:
            if len(cluster_points) < cfg["min_circle_points"]:
                continue

            for _ in range(cfg["ransac_iterations"]):
                if len(cluster_points) < 3:
                    break

                # Sample 3 points for initial fit
                sample_indices = np.random.choice(len(cluster_points), 3, replace=False)
                fit_result = self._fit_circle_algebraic(cluster_points[sample_indices])
                if fit_result is None:
                    continue

                cx, cy, r = fit_result

                # Check if radius is within expected range
                if r < max(0.5, cfg["expected_radius_cm"] - cfg["radius_tolerance_cm"]) or \
                   r > cfg["expected_radius_cm"] + cfg["radius_tolerance_cm"]:
                    continue

                # Find inliers - vectorized distance calculation
                distances_to_circle = np.abs(
                    np.sqrt((cluster_points[:, 0] - cx) ** 2 + (cluster_points[:, 1] - cy) ** 2) - r
                )
                inliers = distances_to_circle < cfg["inlier_threshold_cm"]
                inlier_count = np.sum(inliers)

                if inlier_count < cfg["min_circle_points"]:
                    continue

                # Score based on fit quality and radius match
                score = np.mean(distances_to_circle[inliers]) + abs(r - cfg["expected_radius_cm"]) * 0.2

                if score < best_score:
                    best_score = score
                    inlier_points = cluster_points[inliers]
                    refined_fit = self._fit_circle_algebraic(inlier_points)
                    if refined_fit is not None:
                        best_fit = (*refined_fit, inlier_count)
                        
                        # Early termination if excellent fit found
                        if best_score < early_termination_threshold:
                            return best_fit

        return best_fit


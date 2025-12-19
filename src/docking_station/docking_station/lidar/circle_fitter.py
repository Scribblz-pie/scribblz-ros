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
    
    def __init__(self, logger=None):
        """
        Initialize CircleFitter.
        
        Args:
            logger: Optional ROS2 logger instance for debug output
        """
        self.logger = logger
    
    def find_small_clusters(self, points, min_pts, max_pts, dist_thresh):
        """
        Find clusters of points within a distance threshold.
        
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

        unvisited = set(range(len(points)))
        clusters = []

        while unvisited:
            seed = unvisited.pop()
            cluster_indices = [seed]
            to_check = [seed]

            while to_check:
                current = to_check.pop()
                current_point = points[current]

                unvisited_list = list(unvisited)
                if len(unvisited_list) > 0:
                    unvisited_points = points[unvisited_list]
                    distances = np.linalg.norm(unvisited_points - current_point, axis=1)
                    nearby_mask = distances < dist_thresh

                    for i, is_nearby in enumerate(nearby_mask):
                        if is_nearby:
                            idx = unvisited_list[i]
                            unvisited.discard(idx)
                            cluster_indices.append(idx)
                            to_check.append(idx)

            if min_pts <= len(cluster_indices) <= max_pts:
                clusters.append(points[cluster_indices])

        clusters.sort(key=len)
        return clusters

    def filter_valid_points(self, points, min_dist, max_dist):
        """
        Filter points by distance from origin.
        
        Args:
            points: numpy array of (x, y) points (cm)
            min_dist: Minimum distance from origin (cm)
            max_dist: Maximum distance from origin (cm)
            
        Returns:
            Filtered numpy array of points
        """
        distances = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)
        valid_mask = (distances > min_dist) & (distances < max_dist)
        return points[valid_mask]

    def _fit_circle_algebraic(self, points):
        """
        Fit a circle to points using algebraic least squares.
        
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
            A = np.column_stack([2 * x, 2 * y, np.ones(len(points))])
            b = x ** 2 + y ** 2
            params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            cx, cy, D = params
            r_squared = cx ** 2 + cy ** 2 + D
            if r_squared <= 0:
                return None
            r = math.sqrt(r_squared)
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

        # Convert polar to Cartesian coordinates (in cm)
        points = np.array([(dist * 100.0 * math.cos(math.radians(angle)),
                            dist * 100.0 * math.sin(math.radians(angle)))
                           for angle, dist in zip(angles, distances)])
        
        if self.logger:
            self.logger.debug(f"Input: {len(angles)} points")

        # Filter by distance
        valid_points = self.filter_valid_points(
            points, 
            cfg["min_detection_distance_cm"], 
            cfg["max_detection_distance_cm"]
        )
        if self.logger:
            self.logger.debug(f"After distance filter ({cfg['min_detection_distance_cm']:.1f}-{cfg['max_detection_distance_cm']:.1f} cm): {len(valid_points)} valid points")
        if len(valid_points) < cfg["min_circle_points"]:
            if self.logger:
                self.logger.debug(f"Not enough valid points ({len(valid_points)} < {cfg['min_circle_points']})")
            return None

        # Find clusters
        clusters = self.find_small_clusters(
            valid_points, 
            cfg["min_circle_points"], 
            100,  # max_pts
            10.0  # dist_thresh (cm)
        )
        if self.logger:
            self.logger.debug(f"Found {len(clusters)} clusters")
            if len(clusters) > 0:
                cluster_sizes = [len(c) for c in clusters]
                self.logger.debug(f"Cluster sizes: {cluster_sizes}")
                # Log centroid of each cluster
                for i, cluster in enumerate(clusters):
                    centroid_x = np.mean(cluster[:, 0])
                    centroid_y = np.mean(cluster[:, 1])
                    dist_from_origin = np.sqrt(centroid_x**2 + centroid_y**2)
                    self.logger.debug(f"  Cluster {i}: {len(cluster)} points, centroid=({centroid_x:.2f}, {centroid_y:.2f}) cm, dist={dist_from_origin:.2f} cm")
        
        if len(clusters) == 0:
            if self.logger:
                self.logger.debug("No clusters found")
            return None

        # RANSAC fitting
        best_fit = None
        best_score = float('inf')

        for cluster_idx, cluster_points in enumerate(clusters):
            if len(cluster_points) < cfg["min_circle_points"]:
                continue
            
            if self.logger:
                self.logger.debug(f"Processing cluster {cluster_idx} with {len(cluster_points)} points")

            for iteration in range(cfg["ransac_iterations"]):
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

                # Find inliers
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
                        if self.logger:
                            self.logger.debug(f"  New best fit: center=({refined_fit[0]:.2f}, {refined_fit[1]:.2f}) cm, radius={refined_fit[2]:.2f} cm, inliers={inlier_count}, score={score:.4f}")

        if self.logger:
            if best_fit is not None:
                self.logger.debug(f"Final result: center=({best_fit[0]:.2f}, {best_fit[1]:.2f}) cm, radius={best_fit[2]:.2f} cm, inliers={best_fit[3]}")
            else:
                self.logger.debug("No valid circle fit found")
        
        return best_fit

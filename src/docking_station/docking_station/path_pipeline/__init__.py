"""Drawing robot path planning pipeline.

Modules:
- image_processing: Image to polylines (cv2 + skeletonization)
- stroke_ordering: TSP-based stroke ordering with containment
- orientation_planner: Precomputed tangent-based orientations
- holonomic_motion: Wheel command generation for Kiwi drive
"""

from . import image_processing, stroke_ordering, orientation_planner, holonomic_motion

__all__ = [
    "image_processing",
    "stroke_ordering",
    "orientation_planner",
    "holonomic_motion",
]

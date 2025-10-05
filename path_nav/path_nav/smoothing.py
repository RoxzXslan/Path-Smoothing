
from typing import List, Tuple
import numpy as np
import math
from scipy.interpolate import CubicSpline


def cubic_spline_smoothing(
    points: List[Tuple[float, float]], 
    num_samples: int = 200
) -> Tuple[List[Tuple[float, float]], np.ndarray]:
    """
    Smooth a path using cubic spline interpolation with arc-length parameterization.
    
    This function takes a list of 2D waypoints and generates a smooth curve that
    passes through all points. The curve is C² continuous, ensuring smooth position,
    velocity, and acceleration profiles. Arc-length parameterization ensures uniform
    point distribution regardless of waypoint spacing.
    
    Algorithm:
        1. Compute cumulative arc length at each waypoint
        2. Fit cubic splines x(s) and y(s) where s is arc length
        3. Sample uniformly in arc-length space
    
    Args:
        points: List of (x, y) coordinate tuples representing waypoints.
               Must contain at least 2 unique points.
        num_samples: Number of points to sample along the smoothed curve.
                    Higher values give smoother appearance but slower computation.
                    Default: 200 (suitable for most robotics applications)
    
    Returns:
        tuple containing:
            - smoothed_points: List of (x, y) tuples sampled uniformly along 
                             the smooth curve
            - arc_lengths: Numpy array of arc-length values (in meters) at each 
                          sample point. Useful for velocity profile generation.
    
    Raises:
        ValueError: If points list is empty
        ValueError: If points list has fewer than 2 unique points
        ValueError: If all points are at nearly the same location (< 1mm total path)
        RuntimeError: If scipy is not installed
    
    Example:
        waypoints = [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0), (3.0, 1.0)]
        smooth_path, arc_lengths = cubic_spline_smoothing(waypoints, num_samples=100)
        len(smooth_path)
        100
        arc_lengths[0]  # Should be 0
        0.0
        arc_lengths[-1]  # Total path length
        3.414...
    
    Notes:
        - C² continuity means smooth acceleration (important for robot dynamics)
        - Arc-length parameterization prevents point clustering in straight sections
        - Computational complexity: O(n log n) for spline fitting + O(m) for sampling
    """

    if not points or len(points) == 0:
        raise ValueError("Points list cannot be empty")
    
    if len(points) == 1:
        raise ValueError(
            "Need at least 2 points for cubic spline interpolation. "
            f"Received {len(points)} point(s)."
        )
    
    # Remove duplicate consecutive points
    unique_points = [points[0]]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i-1][0]
        dy = points[i][1] - points[i-1][1]
        dist = math.hypot(dx, dy)
        
        # Only add if dist > 1mm
        if dist > 1e-3:
            unique_points.append(points[i])
    
    if len(unique_points) < 2:
        raise ValueError(
            f"Need at least 2 unique points after removing duplicates. "
            f"Original: {len(points)} points, Unique: {len(unique_points)} points. "
            f"Points may be too close together (< 1mm apart)."
        )
    
    points = unique_points
    
    # Convert to numpy arrays for efficient computation
    xs = np.array([p[0] for p in points], dtype=np.float64)
    ys = np.array([p[1] for p in points], dtype=np.float64)
    
    # Compute cumulative arc length at each waypoint
    # This parameterizes the spline by distance traveled, not by index
    d = np.zeros(len(points))
    for i in range(1, len(points)):
        segment_length = math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1])
        d[i] = d[i-1] + segment_length
    
    # Validate total path length
    if d[-1] < 1e-6:
        raise ValueError(
            f"Total path length is too small: {d[-1]:.2e} meters. "
            f"All waypoints may be at nearly the same location."
        )
    
    try:
        # Fit cubic splines: x(s) and y(s) where s is arc length
        # CubicSpline ensures C² continuity (smooth acceleration)
        csx = CubicSpline(d, xs)
        csy = CubicSpline(d, ys)
    except Exception as e:
        raise RuntimeError(
            f"Failed to create cubic spline. This may indicate "
            f"numerical issues with the waypoints. Error: {str(e)}"
        )
    
    # Sample uniformly in arc-length space
    # This ensures even point distribution along the curve
    s_vals = np.linspace(0.0, d[-1], num_samples)
    
    # Evaluate splines at sample points
    smoothed = [(float(csx(s)), float(csy(s))) for s in s_vals]
    
    return smoothed, s_vals

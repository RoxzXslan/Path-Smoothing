
import math


def compute_arc_lengths(points):
    """
    Compute cumulative arc length at each point along a path.
    
    Args:
        points: List of (x, y) coordinate tuples
    
    Returns:
        List of cumulative distances (first element is 0.0)
    """
    if not points:
        return []
    
    if len(points) == 1:
        return [0.0]
    
    ds = [0.0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i-1][0]
        dy = points[i][1] - points[i-1][1]
        ds.append(ds[-1] + math.hypot(dx, dy))
    return ds


def generate_constant_speed_trajectory(points, v=0.2):
    """
    Generate time-parameterized trajectory with constant velocity.
    
    Args:
        points: List of (x, y) waypoints
        v: Constant linear speed in m/s (default: 0.2)
    
    Returns:
        List of (x, y, t) tuples where t is time in seconds
    
    Raises:
        ValueError: If velocity is zero or negative
    """
    # Validate velocity
    if v <= 0:
        raise ValueError(f"Velocity must be positive. Got v={v}")
    
    # Handle empty list
    if not points:
        return []
    
    # Compute arc lengths
    ds = compute_arc_lengths(points)
    
    # Generate trajectory with timestamps
    traj = []
    for (x, y), s in zip(points, ds):
        t = s / v
        traj.append((x, y, t))
    
    return traj
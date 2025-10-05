"""
Trajectory Visualization Script
Visualizes random waypoints and smoothed path for demo purposes.
"""
import matplotlib.pyplot as plt
import random
from path_nav.smoothing import cubic_spline_smoothing
from path_nav.trajectory_generator import generate_constant_speed_trajectory


def visualize_trajectory():
    """Generate and visualize waypoints with smoothed path."""
    # Generate random waypoints (same logic as ROS2 node)
    random.seed()
    num_points = 10
    waypoints = [
        (random.uniform(0, 7), random.uniform(0, 7)) 
        for _ in range(num_points)
    ]
    waypoints[0] = (0.0, 0.0)  # Start at origin
    waypoints.sort(key=lambda p: p[0])  # Sort by X for forward motion
    
    print(f'Generated waypoints: {waypoints}')
    
    # Smooth and generate trajectory
    try:
        smooth_pts, _ = cubic_spline_smoothing(waypoints, num_samples=200)
        traj = generate_constant_speed_trajectory(smooth_pts, v=0.2)
        print(f'Trajectory generated: {len(traj)} points')
    except Exception as e:
        print(f'Failed to generate trajectory: {e}')
        return
    
    # Create visualization
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Plot waypoints
    waypoint_x = [p[0] for p in waypoints]
    waypoint_y = [p[1] for p in waypoints]
    ax.plot(waypoint_x, waypoint_y, 'ro-', markersize=10, linewidth=2, 
            label='Original Waypoints', alpha=0.6)
    
    # Plot smoothed path
    smooth_x = [p[0] for p in smooth_pts]
    smooth_y = [p[1] for p in smooth_pts]
    ax.plot(smooth_x, smooth_y, 'b-', linewidth=2, 
            label='Smoothed Path (Cubic Spline)')
    
    # Plot trajectory points
    traj_x = [p[0] for p in traj]
    traj_y = [p[1] for p in traj]
    ax.plot(traj_x, traj_y, 'g.', markersize=3, 
            label='Trajectory Points', alpha=0.5)
    
    # Mark start point
    ax.plot(waypoints[0][0], waypoints[0][1], 'g*', 
            markersize=20, label='Start')
    
    # Mark end point
    ax.plot(waypoints[-1][0], waypoints[-1][1], 'r*', 
            markersize=20, label='End')
    
    # Formatting
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.set_title('Trajectory Generation: Waypoints & Smoothed Path', fontsize=14, fontweight='bold')
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.set_xlim(-0.5, 7.5)
    ax.set_ylim(-0.5, 7.5)
    
    plt.tight_layout()
    plt.show()
    
    # Print statistics
    print(f'\nStatistics:')
    print(f'  Waypoints: {len(waypoints)}')
    print(f'  Smoothed points: {len(smooth_pts)}')
    print(f'  Trajectory points: {len(traj)}')


if __name__ == '__main__':
    visualize_trajectory()

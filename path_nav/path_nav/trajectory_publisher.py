"""
Trajectory Publisher Node
Generates random waypoints, smooths them, and publishes as ROS2 Path message.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import random
from path_nav.smoothing import cubic_spline_smoothing
from path_nav.trajectory_generator import generate_constant_speed_trajectory


def quaternion_from_yaw(yaw):
    """Convert yaw angle to quaternion."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class TrajectoryPublisher(Node):
    """
    Publishes smooth trajectory from random waypoints.
    
    Generates random waypoints in a 7x7m space, smooths the path using
    cubic splines, and publishes as nav_msgs/Path message.
    """
    
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        # Publisher
        self.pub = self.create_publisher(Path, 'trajectory', 10)
        
        # Generate random waypoints
        random.seed()
        num_points = 10
        self.waypoints = [
            (random.uniform(0, 7), random.uniform(0, 7)) 
            for _ in range(num_points)
        ]
        self.waypoints[0] = (0.0, 0.0)  # Start at origin
        self.waypoints.sort(key=lambda p: p[0])  # Sort by X for forward motion
        
        self.get_logger().info(f'Generated waypoints: {self.waypoints}')
        
        # Smooth and generate trajectory
        try:
            self.smooth_pts, _ = cubic_spline_smoothing(self.waypoints, num_samples=200)
            self.traj = generate_constant_speed_trajectory(self.smooth_pts, v=0.2)
            self.get_logger().info(f'Trajectory generated: {len(self.traj)} points')
        except Exception as e:
            self.get_logger().error(f'Failed to generate trajectory: {e}')
            self.smooth_pts = []
            self.traj = []
        
        # Timer to publish trajectory once
        self.timer = self.create_timer(1.0, self.publish_trajectory)
        self.published = False
    
    def publish_trajectory(self):
        """Publish the trajectory as a Path message (once)."""
        if self.published or not self.traj:
            return
        
        # Create Path message
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Add each trajectory point
        for i in range(len(self.traj)):
            x, y, t = self.traj[i]
            
            # Compute heading from trajectory direction
            if i < len(self.traj) - 1:
                dx = self.traj[i + 1][0] - x
                dy = self.traj[i + 1][1] - y
                yaw = math.atan2(dy, dx)
            else:
                # Use previous heading for last point
                if i > 0:
                    dx = x - self.traj[i - 1][0]
                    dy = y - self.traj[i - 1][1]
                    yaw = math.atan2(dy, dx)
                else:
                    yaw = 0.0
            
            # Create PoseStamped
            ps = PoseStamped()
            ps.header.stamp = path.header.stamp
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            
            # Set orientation from heading
            q = quaternion_from_yaw(yaw)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            
            path.poses.append(ps)
        
        # Publish
        self.pub.publish(path)
        self.get_logger().info(f'Published trajectory with {len(path.poses)} points')
        self.published = True


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
Pure Pursuit Controller for Differential Drive Robot
Implements geometric path tracking algorithm with adaptive speed control.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math


def yaw_from_quaternion(q):
    """
    Extract yaw angle from quaternion.
    
    Args:
        q: geometry_msgs Quaternion
    
    Returns:
        float: Yaw angle in radians [-pi, pi]
    """
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class PurePursuit(Node):
    """
    Pure Pursuit controller for trajectory tracking.
    
    Subscribes to:
        /trajectory (nav_msgs/Path): Desired trajectory to follow
        /odom (nav_msgs/Odometry): Current robot pose
    
    Publishes:
        /cmd_vel (geometry_msgs/Twist): Velocity commands
    
    Parameters:
        lookahead_distance (float): Lookahead distance in meters (default: 0.4)
        max_velocity (float): Maximum linear velocity in m/s (default: 0.25)
        goal_tolerance (float): Goal reached tolerance in meters (default: 0.1)
        max_angular_velocity (float): Max angular velocity in rad/s (default: 1.5)
    """
    
    def __init__(self):
        super().__init__('pure_pursuit')
        
        # Declare parameters for easy tuning
        self.declare_parameter('lookahead_distance', 0.4)
        self.declare_parameter('max_velocity', 0.25)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('max_angular_velocity', 1.5)
        
        # Get parameters
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.max_vel = self.get_parameter('max_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_w = self.get_parameter('max_angular_velocity').value
        
        # Log parameters
        self.get_logger().info(f'Pure Pursuit Controller Initialized')
        self.get_logger().info(f'  Lookahead: {self.lookahead}m')
        self.get_logger().info(f'  Max velocity: {self.max_vel}m/s')
        self.get_logger().info(f'  Goal tolerance: {self.goal_tolerance}m')
        self.get_logger().info(f'  Max angular velocity: {self.max_w}rad/s')
        
        # Subscriptions
        self.sub_path = self.create_subscription(
            Path, 'trajectory', self.path_cb, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 20
        )
        
        # Publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.trajectory = []  # List of (x, y) tuples
        self.current_pose = None  # (x, y, yaw)
        self.goal_reached = False
        
        # Control loop at 20Hz
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Waiting for trajectory and odometry...')
    
    def path_cb(self, msg: Path):
        """
        Callback for trajectory messages.
        
        Args:
            msg (nav_msgs/Path): Trajectory to follow
        """
        pts = []
        for ps in msg.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            pts.append((x, y))
        
        self.trajectory = pts
        self.goal_reached = False  # Reset for new trajectory
        self.get_logger().info(f'Received trajectory with {len(pts)} points')
    
    def odom_cb(self, msg: Odometry):
        """
        Callback for odometry messages.
        
        Args:
            msg (nav_msgs/Odometry): Current robot pose
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_pose = (x, y, yaw)
    
    def find_lookahead_point(self, x, y):
        """
        Find the lookahead point on the trajectory.
        
        Searches for the point on the trajectory that is closest to the
        lookahead distance from the current position.
        
        Args:
            x (float): Current x position
            y (float): Current y position
        
        Returns:
            tuple: (lookahead_x, lookahead_y, distance) or None if no trajectory
        """
        if not self.trajectory:
            return None
        
        # Find point closest to lookahead distance
        best_point = None
        best_diff = float('inf')
        
        for px, py in self.trajectory:
            d = math.hypot(px - x, py - y)
            diff = abs(d - self.lookahead)
            
            if diff < best_diff:
                best_diff = diff
                best_point = (px, py, d)
            
            # If we've passed lookahead and found a good point, use it
            if d > self.lookahead and best_point is not None:
                break
        
        # If no suitable point found, return goal
        if best_point is None:
            goal = self.trajectory[-1]
            dist_to_goal = math.hypot(goal[0] - x, goal[1] - y)
            return (goal[0], goal[1], dist_to_goal)
        
        return best_point
    
    def control_loop(self):
        """
        Main control loop executed at fixed rate.
        
        Implements Pure Pursuit algorithm:
        1. Check if goal is reached
        2. Find lookahead point
        3. Calculate curvature to lookahead point
        4. Compute velocity commands
        5. Publish commands
        """
        # Check if we have necessary data
        if self.current_pose is None or not self.trajectory:
            return
        
        # Check if already reached goal
        if self.goal_reached:
            return
        
        x, y, yaw = self.current_pose
        
        # Check if goal reached (do this FIRST)
        goal = self.trajectory[-1]
        dist_to_goal = math.hypot(goal[0] - x, goal[1] - y)
        
        if dist_to_goal < self.goal_tolerance:
            # Stop the robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd.publish(twist)
            
            self.get_logger().info(
                f'Goal reached! Final error: {dist_to_goal:.3f}m'
            )
            self.goal_reached = True
            return
        
        # Find lookahead point
        look = self.find_lookahead_point(x, y)
        if look is None:
            self.get_logger().warn('No lookahead point found')
            return
        
        lx, ly, d = look
        
        # Calculate angle to lookahead point
        angle_to_point = math.atan2(ly - y, lx - x)
        alpha = self._angle_diff(angle_to_point, yaw)
        
        # Pure Pursuit: curvature = 2*sin(alpha)/L
        curvature = 2.0 * math.sin(alpha) / max(self.lookahead, 1e-6)
        
        # Speed modulation based on curvature (slow down on turns)
        v = self.max_vel * (1.0 / (1.0 + abs(curvature) * 2.0))
        v = max(0.0, v)  # Ensure non-negative
        
        # Angular velocity from curvature
        w = v * curvature
        
        # Saturate angular velocity
        if abs(w) > self.max_w:
            w = math.copysign(self.max_w, w)
        
        # Publish velocity command
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.pub_cmd.publish(twist)
        
        # Optional: Log progress periodically
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 100 == 0:  # Every 5 seconds at 20Hz
            self.get_logger().info(
                f'Tracking: dist_to_goal={dist_to_goal:.2f}m, '
                f'v={v:.2f}m/s, w={w:.2f}rad/s'
            )
    
    def _angle_diff(self, a, b):
        """
        Compute the smallest angle difference between two angles.
        
        Args:
            a (float): First angle in radians
            b (float): Second angle in radians
        
        Returns:
            float: Angle difference in radians, normalized to [-pi, pi]
        """
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = PurePursuit()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure robot stops on shutdown
        twist = Twist()
        node.pub_cmd.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
Unit tests for trajectory generation
"""
import unittest
import math
import sys

from path_nav.trajectory_generator import (
    compute_arc_lengths,
    generate_constant_speed_trajectory
)


class TestArcLengthComputation(unittest.TestCase):
    """Test arc length computation"""
    
    def test_straight_line_unit_distance(self):
        """Test arc length for unit distance straight line"""
        points = [(0.0, 0.0), (1.0, 0.0)]
        arc_lengths = compute_arc_lengths(points)
        
        self.assertEqual(len(arc_lengths), 2)
        self.assertAlmostEqual(arc_lengths[0], 0.0, places=5)
        self.assertAlmostEqual(arc_lengths[1], 1.0, places=5)
        
        print("✓ Unit distance test passed")
    
    def test_straight_line_multiple_points(self):
        """Test arc length for multiple collinear points"""
        points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
        arc_lengths = compute_arc_lengths(points)
        
        expected = [0.0, 1.0, 2.0, 3.0]
        for computed, expected_val in zip(arc_lengths, expected):
            self.assertAlmostEqual(computed, expected_val, places=5)
        
        print("✓ Multiple points test passed")
    
    def test_diagonal_line(self):
        """Test arc length for diagonal line (3-4-5 triangle)"""
        points = [(0.0, 0.0), (3.0, 4.0)]
        arc_lengths = compute_arc_lengths(points)
        
        self.assertAlmostEqual(arc_lengths[0], 0.0, places=5)
        self.assertAlmostEqual(arc_lengths[1], 5.0, places=5)
        
        print("✓ Diagonal line test passed")
    
    def test_l_shape(self):
        """Test arc length for L-shaped path"""
        points = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]
        arc_lengths = compute_arc_lengths(points)
        
        self.assertAlmostEqual(arc_lengths[0], 0.0, places=5)
        self.assertAlmostEqual(arc_lengths[1], 3.0, places=5)
        self.assertAlmostEqual(arc_lengths[2], 7.0, places=5)
        
        print("✓ L-shape test passed")
    
    def test_single_point(self):
        """Test arc length with single point"""
        points = [(1.0, 2.0)]
        arc_lengths = compute_arc_lengths(points)
        
        self.assertEqual(len(arc_lengths), 1)
        self.assertAlmostEqual(arc_lengths[0], 0.0, places=5)
        
        print("✓ Single point test passed")
    
    def test_monotonic_increase(self):
        """Test that arc lengths always increase"""
        points = [(0.0, 0.0), (1.0, 1.0), (2.5, 0.5), (4.0, 3.0)]
        arc_lengths = compute_arc_lengths(points)
        
        for i in range(len(arc_lengths) - 1):
            self.assertLessEqual(arc_lengths[i], arc_lengths[i+1],
                                "Arc length should be monotonically increasing")
        
        print("✓ Monotonic increase test passed")
    
    def test_assignment_path(self):
        """Test with actual assignment waypoints"""
        points = [(0,0), (1,0), (2,1), (3,1.5), (4,0.5), (5,0)]
        arc_lengths = compute_arc_lengths(points)
        
        # Should have 6 values
        self.assertEqual(len(arc_lengths), 6)
        
        # First should be 0
        self.assertAlmostEqual(arc_lengths[0], 0.0)
        
        # Last should be total path length
        total_length = arc_lengths[-1]
        self.assertGreater(total_length, 5.0)  # At least as long as straight line
        
        print(f"✓ Assignment path test passed (length: {total_length:.2f}m)")


class TestConstantSpeedTrajectory(unittest.TestCase):
    """Test constant speed trajectory generation"""
    
    def test_basic_trajectory(self):
        """Test basic trajectory generation"""
        points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        v = 0.5  # m/s
        
        trajectory = generate_constant_speed_trajectory(points, v)
        
        self.assertEqual(len(trajectory), 3)
        
        # Check format (x, y, t)
        for traj_point in trajectory:
            self.assertEqual(len(traj_point), 3)
        
        print("✓ Basic trajectory test passed")
    
    def test_time_calculation(self):
        """Test correct time calculation"""
        points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        v = 1.0  # m/s
        
        trajectory = generate_constant_speed_trajectory(points, v)
        
        # Expected times: 0, 1, 2 seconds
        self.assertAlmostEqual(trajectory[0][2], 0.0, places=5)
        self.assertAlmostEqual(trajectory[1][2], 1.0, places=5)
        self.assertAlmostEqual(trajectory[2][2], 2.0, places=5)
        
        print("✓ Time calculation test passed")
    
    def test_velocity_scaling(self):
        """Test that time scales correctly with velocity"""
        points = [(0.0, 0.0), (10.0, 0.0)]
        
        traj_v1 = generate_constant_speed_trajectory(points, v=1.0)
        traj_v2 = generate_constant_speed_trajectory(points, v=2.0)
        
        # At v=2.0, should take half the time
        self.assertAlmostEqual(traj_v2[1][2], traj_v1[1][2] / 2.0, places=5)
        
        print("✓ Velocity scaling test passed")
    
    def test_positions_preserved(self):
        """Test that positions are preserved in trajectory"""
        points = [(1.5, 2.3), (4.7, 5.1), (7.2, 3.8)]
        trajectory = generate_constant_speed_trajectory(points, v=0.5)
        
        for i, (x, y, t) in enumerate(trajectory):
            self.assertAlmostEqual(x, points[i][0], places=5)
            self.assertAlmostEqual(y, points[i][1], places=5)
        
        print("✓ Position preservation test passed")
    
    def test_time_monotonic(self):
        """Test that time increases monotonically"""
        points = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.5), (3.0, 2.0)]
        trajectory = generate_constant_speed_trajectory(points, v=0.5)
        
        times = [t for _, _, t in trajectory]
        for i in range(len(times) - 1):
            self.assertLess(times[i], times[i+1],
                           "Time should increase monotonically")
        
        print("✓ Time monotonic test passed")
    
    def test_start_time_zero(self):
        """Test that trajectory starts at t=0"""
        points = [(5.0, 5.0), (10.0, 10.0)]
        trajectory = generate_constant_speed_trajectory(points, v=1.0)
        
        self.assertAlmostEqual(trajectory[0][2], 0.0, places=5,
                              msg="Trajectory should start at t=0")
        
        print("✓ Start time test passed")
    
    def test_different_speeds(self):
        """Test trajectory generation with different speeds"""
        points = [(0.0, 0.0), (1.0, 0.0)]
        
        speeds = [0.1, 0.5, 1.0, 2.0]
        for v in speeds:
            trajectory = generate_constant_speed_trajectory(points, v)
            expected_time = 1.0 / v
            self.assertAlmostEqual(trajectory[1][2], expected_time, places=5,
                                  msg=f"Failed for speed {v}")
        
        print("✓ Different speeds test passed")
    
    def test_complex_path(self):
        """Test with more complex path"""
        points = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0), (0.0, 4.0)]
        v = 0.5
        
        trajectory = generate_constant_speed_trajectory(points, v)
        
        # Total distance: 3 + 4 + 3 = 10m
        # At v=0.5 m/s, should take 20 seconds
        self.assertAlmostEqual(trajectory[-1][2], 20.0, places=3)
        
        print("✓ Complex path test passed")
    
    def test_single_point_trajectory(self):
        """Test trajectory with single point"""
        points = [(1.0, 1.0)]
        trajectory = generate_constant_speed_trajectory(points, v=0.5)
        
        self.assertEqual(len(trajectory), 1)
        self.assertAlmostEqual(trajectory[0][2], 0.0, places=5)
        
        print("✓ Single point trajectory test passed")
    
    def test_default_velocity(self):
        """Test with default velocity (0.2 m/s from assignment)"""
        points = [(0.0, 0.0), (1.0, 0.0)]
        trajectory = generate_constant_speed_trajectory(points)  # uses v=0.2 default
        
        # At v=0.2, 1m should take 5 seconds
        self.assertAlmostEqual(trajectory[1][2], 5.0, places=5)
        
        print("✓ Default velocity test passed")
    
    def test_assignment_trajectory(self):
        """Test with actual assignment waypoints"""
        # These are after smoothing in your code
        points = [(0,0), (1,0), (2,1), (3,1.5), (4,0.5), (5,0)]
        v = 0.2  # Your default velocity
        
        trajectory = generate_constant_speed_trajectory(points, v)
        
        # Check basic properties
        self.assertEqual(len(trajectory), len(points))
        self.assertAlmostEqual(trajectory[0][2], 0.0)
        self.assertGreater(trajectory[-1][2], 0.0)
        
        # Verify all times are unique and increasing
        times = [t for _, _, t in trajectory]
        self.assertEqual(times, sorted(times))
        self.assertEqual(len(times), len(set(times)))  # All unique
        
        print(f"✓ Assignment trajectory test passed (duration: {trajectory[-1][2]:.2f}s)")


class TestEdgeCases(unittest.TestCase):
    """Test edge cases and error conditions"""
    
    def test_zero_distance_segment(self):
        """Test with duplicate consecutive points (zero distance)"""
        points = [(0.0, 0.0), (1.0, 1.0), (1.0, 1.0), (2.0, 2.0)]
        trajectory = generate_constant_speed_trajectory(points, v=1.0)
        
        # Times for duplicate point should be same
        self.assertAlmostEqual(trajectory[1][2], trajectory[2][2], places=5)
        
        print("✓ Zero distance segment test passed")
    
    def test_very_small_velocity(self):
        """Test with very small velocity"""
        points = [(0.0, 0.0), (0.01, 0.0)]
        trajectory = generate_constant_speed_trajectory(points, v=0.01)
        
        # Should still work, just take longer
        self.assertAlmostEqual(trajectory[1][2], 1.0, places=3)
        
        print("✓ Small velocity test passed")
    
    def test_very_high_velocity(self):
        """Test with very high velocity"""
        points = [(0.0, 0.0), (1.0, 0.0)]
        trajectory = generate_constant_speed_trajectory(points, v=100.0)
        
        # Time should be very small but positive
        self.assertGreater(trajectory[1][2], 0.0)
        self.assertLess(trajectory[1][2], 0.1)
        
        print("✓ High velocity test passed")
    
    def test_negative_coordinates(self):
        """Test with negative coordinates"""
        points = [(-1.0, -1.0), (0.0, 0.0), (1.0, 1.0)]
        trajectory = generate_constant_speed_trajectory(points, v=0.5)
        
        # Should handle negative coordinates fine
        self.assertEqual(len(trajectory), 3)
        self.assertLess(trajectory[0][0], 0.0)
        
        print("✓ Negative coordinates test passed")
    
    def test_large_distances(self):
        """Test with very large distances"""
        points = [(0.0, 0.0), (1000.0, 0.0)]
        trajectory = generate_constant_speed_trajectory(points, v=1.0)
        
        # Should take 1000 seconds
        self.assertAlmostEqual(trajectory[1][2], 1000.0, places=3)
        
        print("✓ Large distances test passed")
    
    def test_empty_points_list(self):
        """Test with empty points list"""
        points = []
        trajectory = generate_constant_speed_trajectory(points, v=0.5)
        
        # Should return empty list
        self.assertEqual(len(trajectory), 0)
        
        print("✓ Empty points list test passed")


class TestIntegration(unittest.TestCase):
    """Integration tests combining smoothing and trajectory generation"""
    
    def test_full_pipeline(self):
        """Test complete pipeline: waypoints -> smoothing -> trajectory"""
        from path_nav.smoothing import cubic_spline_smoothing
        
        # Start with coarse waypoints
        waypoints = [(0,0), (1,0), (2,1), (3,1.5), (4,0.5), (5,0)]
        
        # Smooth the path
        smooth_pts, s_vals = cubic_spline_smoothing(waypoints, num_samples=200)
        
        # Generate trajectory
        trajectory = generate_constant_speed_trajectory(smooth_pts, v=0.2)
        
        # Verify output
        self.assertEqual(len(trajectory), 200)
        self.assertAlmostEqual(trajectory[0][2], 0.0)
        self.assertGreater(trajectory[-1][2], 0.0)
        
        print(f"✓ Full pipeline test passed (200 points, {trajectory[-1][2]:.2f}s duration)")
    
    def test_pipeline_different_samples(self):
        """Test pipeline with different sample counts"""
        from path_nav.smoothing import cubic_spline_smoothing
        
        waypoints = [(0,0), (1,1), (2,0)]
        
        for num_samples in [50, 100, 200]:
            smooth_pts, _ = cubic_spline_smoothing(waypoints, num_samples=num_samples)
            trajectory = generate_constant_speed_trajectory(smooth_pts, v=0.5)
            
            self.assertEqual(len(trajectory), num_samples)
            print(f"  ✓ {num_samples} samples: {trajectory[-1][2]:.2f}s")
        
        print("✓ Different samples test passed")
    
    def test_pipeline_consistency(self):
        """Test that pipeline produces consistent results"""
        from path_nav.smoothing import cubic_spline_smoothing
        
        waypoints = [(0,0), (2,2), (4,0)]
        
        # Run twice
        smooth1, _ = cubic_spline_smoothing(waypoints, num_samples=100)
        traj1 = generate_constant_speed_trajectory(smooth1, v=0.3)
        
        smooth2, _ = cubic_spline_smoothing(waypoints, num_samples=100)
        traj2 = generate_constant_speed_trajectory(smooth2, v=0.3)
        
        # Should produce identical results
        self.assertEqual(len(traj1), len(traj2))
        self.assertAlmostEqual(traj1[-1][2], traj2[-1][2], places=5)
        
        print("✓ Pipeline consistency test passed")


class TestRealWorldScenarios(unittest.TestCase):
    """Test realistic robotics scenarios"""
    
    def test_turtlebot_typical_path(self):
        """Test typical TurtleBot3 navigation scenario"""
        # Typical indoor path
        points = [(0.0, 0.0), (1.5, 0.0), (1.5, 1.0), (3.0, 1.0), (3.0, 0.0)]
        v = 0.22  # TurtleBot3 typical speed
        
        trajectory = generate_constant_speed_trajectory(points, v)
        
        # Calculate total distance
        total_dist = 0.0
        for i in range(len(points) - 1):
            dx = points[i+1][0] - points[i][0]
            dy = points[i+1][1] - points[i][1]
            total_dist += math.hypot(dx, dy)
        
        expected_time = total_dist / v
        self.assertAlmostEqual(trajectory[-1][2], expected_time, places=3)
        
        print(f"✓ TurtleBot path test passed ({total_dist:.2f}m in {expected_time:.2f}s)")
    
    def test_slow_precise_motion(self):
        """Test slow, precise motion for tight spaces"""
        points = [(0.0, 0.0), (0.5, 0.0), (0.5, 0.5)]
        v = 0.05  # Very slow
        
        trajectory = generate_constant_speed_trajectory(points, v)
        
        # Should take considerable time
        self.assertGreater(trajectory[-1][2], 10.0)
        
        print(f"✓ Slow motion test passed ({trajectory[-1][2]:.2f}s)")
    
    def test_fast_corridor_navigation(self):
        """Test faster motion in open corridor"""
        points = [(0.0, 0.0), (10.0, 0.0)]  # Long straight corridor
        v = 0.5  # Faster speed
        
        trajectory = generate_constant_speed_trajectory(points, v)
        
        self.assertAlmostEqual(trajectory[-1][2], 20.0, places=3)
        
        print("✓ Fast corridor test passed")


class TestPerformance(unittest.TestCase):
    """Test performance characteristics"""
    
    def test_large_trajectory(self):
        """Test with large number of trajectory points"""
        import time
        
        # Generate 1000 points
        points = [(float(i*0.1), float((i % 10)*0.1)) for i in range(1000)]
        
        start_time = time.time()
        trajectory = generate_constant_speed_trajectory(points, v=0.2)
        elapsed = time.time() - start_time
        
        self.assertEqual(len(trajectory), 1000)
        self.assertLess(elapsed, 0.5, "Should complete quickly")
        
        print(f"✓ Large trajectory test passed ({elapsed*1000:.2f}ms for 1000 points)")
    
    def test_repeated_generation(self):
        """Test repeated trajectory generation (typical in ROS loop)"""
        import time
        
        points = [(0,0), (1,1), (2,0), (3,1), (4,0)]
        
        start_time = time.time()
        for _ in range(100):
            trajectory = generate_constant_speed_trajectory(points, v=0.2)
        elapsed = time.time() - start_time
        
        avg_time = elapsed / 100.0
        self.assertLess(avg_time, 0.01, "Should be fast enough for real-time")
        
        print(f"✓ Repeated generation test passed ({avg_time*1000:.3f}ms avg)")


def run_all_tests():
    """Run all test suites"""
    print("\n" + "="*70)
    print("Running Trajectory Generator Tests")
    print("="*70 + "\n")
    
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestArcLengthComputation))
    suite.addTests(loader.loadTestsFromTestCase(TestConstantSpeedTrajectory))
    suite.addTests(loader.loadTestsFromTestCase(TestEdgeCases))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegration))
    suite.addTests(loader.loadTestsFromTestCase(TestRealWorldScenarios))
    suite.addTests(loader.loadTestsFromTestCase(TestPerformance))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*70)
    print("Test Summary")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.wasSuccessful():
        print("\n✓ ALL TESTS PASSED!")
    else:
        print("\n✗ SOME TESTS FAILED")
    
    print("="*70 + "\n")
    
    return result.wasSuccessful()


if __name__ == '__main__':
    # Run all tests with detailed output
    success = run_all_tests()
    sys.exit(0 if success else 1)
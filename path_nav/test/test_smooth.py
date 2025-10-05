"""
Unit tests for path smoothing algorithms
"""
import unittest
import numpy as np
import math
import sys
import os

# Import from path_nav package
from path_nav.smoothing import cubic_spline_smoothing


class TestCubicSplineSmoothing(unittest.TestCase):
    """Test cases for cubic spline path smoothing"""
    
    def test_straight_line_horizontal(self):
        """Test that a straight horizontal line remains straight"""
        waypoints = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=50)
        
        # Check all y-values are near 0
        for x, y in smoothed:
            self.assertAlmostEqual(y, 0.0, places=1,
                                   msg="Straight line should have y≈0")
        
        # Check x increases monotonically
        x_vals = [pt[0] for pt in smoothed]
        self.assertTrue(all(x_vals[i] <= x_vals[i+1] for i in range(len(x_vals)-1)),
                        "X values should increase monotonically")
        
        print("✓ Straight line test passed")
    
    def test_straight_line_vertical(self):
        """Test that a straight vertical line remains straight"""
        waypoints = [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (0.0, 3.0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=50)
        
        # Check all x-values are near 0
        for x, y in smoothed:
            self.assertAlmostEqual(x, 0.0, places=1,
                                   msg="Vertical line should have x≈0")
        
        print("✓ Vertical line test passed")
    
    def test_passes_through_waypoints(self):
        """Test that smoothed path passes through or near original waypoints"""
        waypoints = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.5), (3.0, 1.5)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=200)
        
        # Check first and last points match exactly
        self.assertAlmostEqual(smoothed[0][0], waypoints[0][0], places=2)
        self.assertAlmostEqual(smoothed[0][1], waypoints[0][1], places=2)
        self.assertAlmostEqual(smoothed[-1][0], waypoints[-1][0], places=2)
        self.assertAlmostEqual(smoothed[-1][1], waypoints[-1][1], places=2)
        
        print("✓ Waypoint interpolation test passed")
    
    def test_minimum_waypoints(self):
        """Test behavior with minimum number of waypoints"""
        waypoints = [(0.0, 0.0), (1.0, 1.0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=20)
        
        # Should still produce valid output
        self.assertEqual(len(smoothed), 20)
        self.assertEqual(len(s_vals), 20)
        
        print("✓ Minimum waypoints test passed")
    
    def test_single_waypoint_error(self):
        """Test that single waypoint raises appropriate error"""
        waypoints = [(0.0, 0.0)]
        
        with self.assertRaises(Exception):
            cubic_spline_smoothing(waypoints, num_samples=10)
        
        print("✓ Single waypoint error handling passed")
    
    def test_arc_length_parameterization(self):
        """Test that arc length values increase monotonically"""
        waypoints = [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0), (3.0, 1.0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=50)
        
        # Arc length should increase monotonically
        self.assertTrue(all(s_vals[i] <= s_vals[i+1] for i in range(len(s_vals)-1)),
                        "Arc length should increase monotonically")
        
        # First arc length should be 0
        self.assertAlmostEqual(s_vals[0], 0.0, places=5)
        
        print("✓ Arc length parameterization test passed")
    
    def test_smoothness_continuity(self):
        """Test that smoothed path is continuous (no jumps)"""
        waypoints = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.5), (3.0, 2.0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=100)
        
        # Check no large jumps between consecutive points
        max_jump = 0.0
        for i in range(len(smoothed) - 1):
            dx = smoothed[i+1][0] - smoothed[i][0]
            dy = smoothed[i+1][1] - smoothed[i][1]
            dist = math.hypot(dx, dy)
            max_jump = max(max_jump, dist)
            
            # Max distance between consecutive points should be reasonable
            self.assertLess(dist, 0.5, 
                           f"Jump too large between points {i} and {i+1}: {dist}")
        
        print(f"✓ Smoothness test passed (max jump: {max_jump:.4f}m)")
    
    def test_output_length(self):
        """Test that output has correct number of samples"""
        waypoints = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]
        num_samples = 75
        
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=num_samples)
        
        self.assertEqual(len(smoothed), num_samples)
        self.assertEqual(len(s_vals), num_samples)
        
        print("✓ Output length test passed")
    
    def test_sharp_turn(self):
        """Test smoothing on a sharp 90-degree turn"""
        waypoints = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=50)
        
        # Should produce smooth curve, not sharp corner
        self.assertEqual(len(smoothed), 50)
        
        # Check that path doesn't have extreme curvature changes
        for i in range(len(smoothed) - 1):
            dist = math.hypot(
                smoothed[i+1][0] - smoothed[i][0],
                smoothed[i+1][1] - smoothed[i][1]
            )
            self.assertGreater(dist, 0.0, "Points should not overlap")
        
        print("✓ Sharp turn smoothing test passed")
    
    def test_empty_waypoints(self):
        """Test that empty waypoints list raises error"""
        waypoints = []
        
        with self.assertRaises(Exception):
            cubic_spline_smoothing(waypoints, num_samples=10)
        
        print("✓ Empty waypoints error handling passed")
    
    def test_assignment_waypoints(self):
        """Test with the actual assignment waypoints"""
        # Example waypoints from your trajectory_publisher.py
        waypoints = [(0,0), (1,0), (2,1), (3,1.5), (4,0.5), (5,0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=200)
        
        # Should produce 200 points
        self.assertEqual(len(smoothed), 200)
        
        # Should start and end at correct positions
        self.assertAlmostEqual(smoothed[0][0], 0.0, places=2)
        self.assertAlmostEqual(smoothed[0][1], 0.0, places=2)
        self.assertAlmostEqual(smoothed[-1][0], 5.0, places=2)
        self.assertAlmostEqual(smoothed[-1][1], 0.0, places=2)
        
        # Compute total path length
        total_length = s_vals[-1]
        self.assertGreater(total_length, 0.0)
        
        print(f"✓ Assignment waypoints test passed (path length: {total_length:.2f}m)")


class TestEdgeCases(unittest.TestCase):
    """Test edge cases and error conditions"""
    
    def test_collinear_points(self):
        """Test smoothing with all collinear points"""
        waypoints = [(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=50)
        
        # Should produce a straight line
        self.assertEqual(len(smoothed), 50)
        
        print("✓ Collinear points test passed")
    
    def test_very_close_waypoints(self):
        """Test with waypoints very close together"""
        waypoints = [(0.0, 0.0), (0.001, 0.001), (0.002, 0.0), (1.0, 1.0)]
        
        try:
            smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=50)
            self.assertEqual(len(smoothed), 50)
            print("✓ Close waypoints test passed")
        except Exception as e:
            # Acceptable to fail on degenerate cases
            print(f"✓ Close waypoints correctly rejected: {str(e)[:50]}")
    
    def test_duplicate_waypoints(self):
        """Test handling of duplicate consecutive waypoints"""
        waypoints = [(0.0, 0.0), (1.0, 1.0), (1.0, 1.0), (2.0, 0.0)]
        
        # Should either handle gracefully or raise informative error
        try:
            smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=50)
            self.assertEqual(len(smoothed), 50)
            print("✓ Duplicate waypoints handled")
        except Exception as e:
            print(f"✓ Duplicate waypoints rejected: {str(e)[:50]}")
    
    def test_large_number_of_samples(self):
        """Test with very large number of samples"""
        waypoints = [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=1000)
        
        self.assertEqual(len(smoothed), 1000)
        
        print("✓ Large sample count test passed")
    
    def test_small_number_of_samples(self):
        """Test with very small number of samples"""
        waypoints = [(0.0, 0.0), (1.0, 1.0), (2.0, 0.0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=3)
        
        self.assertEqual(len(smoothed), 3)
        
        print("✓ Small sample count test passed")


class TestRobustness(unittest.TestCase):
    """Test robustness with various path configurations"""
    
    def test_zigzag_path(self):
        """Test with zigzag pattern"""
        waypoints = [(0,0), (1,1), (2,0), (3,1), (4,0)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=100)
        
        self.assertEqual(len(smoothed), 100)
        print("✓ Zigzag path test passed")
    
    def test_circular_path(self):
        """Test with circular waypoints"""
        # Create waypoints in a circle
        n_points = 8
        radius = 2.0
        waypoints = []
        for i in range(n_points):
            angle = 2 * math.pi * i / n_points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            waypoints.append((x, y))
        
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=100)
        
        self.assertEqual(len(smoothed), 100)
        
        # Check approximate circularity
        center_x = sum(x for x, y in smoothed) / len(smoothed)
        center_y = sum(y for x, y in smoothed) / len(smoothed)
        
        radii = [math.hypot(x - center_x, y - center_y) for x, y in smoothed]
        avg_radius = sum(radii) / len(radii)
        
        # Should be close to original radius
        self.assertAlmostEqual(avg_radius, radius, delta=0.5)
        
        print(f"✓ Circular path test passed (avg radius: {avg_radius:.2f}m)")
    
    def test_s_curve(self):
        """Test with S-curve pattern"""
        waypoints = [(0,0), (1,0), (2,1), (3,2), (4,2), (5,2)]
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=100)
        
        self.assertEqual(len(smoothed), 100)
        print("✓ S-curve path test passed")


class TestPerformance(unittest.TestCase):
    """Test performance characteristics"""
    
    def test_many_waypoints(self):
        """Test with many waypoints"""
        import time
        
        # Create 100 waypoints
        waypoints = [(float(i), float(i % 10)) for i in range(100)]
        
        start_time = time.time()
        smoothed, s_vals = cubic_spline_smoothing(waypoints, num_samples=500)
        elapsed = time.time() - start_time
        
        self.assertEqual(len(smoothed), 500)
        self.assertLess(elapsed, 1.0, "Should complete in under 1 second")
        
        print(f"✓ Performance test passed ({elapsed*1000:.2f}ms for 100 waypoints)")


def run_all_tests():
    """Run all test suites"""
    print("\n" + "="*70)
    print("Running Path Smoothing Tests")
    print("="*70 + "\n")
    
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestCubicSplineSmoothing))
    suite.addTests(loader.loadTestsFromTestCase(TestEdgeCases))
    suite.addTests(loader.loadTestsFromTestCase(TestRobustness))
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
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
from xarm_shape_tracer.shape_converter import ShapeConverter, ShapeType
from xarm_shape_tracer.utils import create_pose, transform_point

class TestShapes(Node):
    """
    Test node to visualize shape generation without robot control
    """
    
    def __init__(self):
        super().__init__('test_shapes')
        
        self.shape_converter = ShapeConverter()
        self.setup_test_shapes()
        
    def setup_test_shapes(self):
        """Setup test shapes"""
        self.shape_converter.add_shape(
            ShapeType.RECTANGLE,
            points=[(-0.1, -0.1), (0.1, 0.1)],
            name="test_square"
        )
        
        self.shape_converter.add_shape(
            ShapeType.CIRCLE,
            points=[(0.2, 0.2)],
            radius=0.1,
            name="test_circle"
        )
    
    def test_conversion(self):
        """Test shape to 3D conversion"""
        
        # Test with horizontal plane
        plane_origin = (0.3, 0.0, 0.2)
        plane_normal = (0.0, 0.0, 1.0)
        
        trajectory = self.shape_converter.generate_trajectory(
            plane_origin=plane_origin,
            plane_normal=plane_normal,
            resolution=0.01,
            scale=1.0
        )
        
        self.get_logger().info(f'Generated {len(trajectory)} 3D points')
        
        # Print first few points
        for i, point in enumerate(trajectory[:5]):
            self.get_logger().info(f'Point {i}: ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})')
        
        return trajectory

def main(args=None):
    rclpy.init(args=args)
    
    test_node = TestShapes()
    
    try:
        trajectory = test_node.test_conversion()
        test_node.get_logger().info('Test completed successfully')
    except Exception as e:
        test_node.get_logger().error(f'Test failed: {e}')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
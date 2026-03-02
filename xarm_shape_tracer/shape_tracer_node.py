#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
from typing import List, Tuple
import yaml
import os

from xarm_shape_tracer.shape_converter import ShapeConverter, ShapeType
from xarm_shape_tracer.trajectory_planner import TrajectoryPlanner
from xarm_shape_tracer.utils import create_pose

class ShapeTracerNode(Node):
    """
    Main node for tracing shapes with xArm7
    """
    
    def __init__(self):
        super().__init__('shape_tracer_node')
        
        # Load parameters
        self.declare_parameter('plane_origin', [0.3, 0.0, 0.2])
        self.declare_parameter('plane_normal', [0.0, 0.0, 1.0])
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('resolution', 0.005)  # 5mm resolution
        self.declare_parameter('approach_height', 0.1)  # Height above plane for approach
        
        self.plane_origin = self.get_parameter('plane_origin').value
        self.plane_normal = self.get_parameter('plane_normal').value
        self.scale = self.get_parameter('scale').value
        self.resolution = self.get_parameter('resolution').value
        self.approach_height = self.get_parameter('approach_height').value
        
        # Initialize components
        self.shape_converter = ShapeConverter()
        self.trajectory_planner = TrajectoryPlanner()
        
        # Define shapes (example)
        self.setup_shapes()
        
        self.get_logger().info('Shape Tracer Node initialized')
    
    def setup_shapes(self):
        """Define the shapes to trace"""
        
        # Square
        self.shape_converter.add_shape(
            ShapeType.RECTANGLE,
            points=[(-0.1, -0.1), (0.1, 0.1)],
            name="square"
        )
        
        # Circle
        self.shape_converter.add_shape(
            ShapeType.CIRCLE,
            points=[(0.0, 0.0)],
            radius=0.15,
            name="circle"
        )
        
        # Triangle
        self.shape_converter.add_shape(
            ShapeType.POLYGON,
            points=[(0.0, 0.2), (0.15, -0.1), (-0.15, -0.1)],
            name="triangle"
        )
        
        # Line
        self.shape_converter.add_shape(
            ShapeType.LINE,
            points=[(-0.2, 0.2), (0.2, -0.2)],
            name="diagonal_line"
        )
    
    def generate_poses(self) -> List[Pose]:
        """Generate pose waypoints from shapes"""
        
        # Get 3D trajectory points
        trajectory_points = self.shape_converter.generate_trajectory(
            plane_origin=tuple(self.plane_origin),
            plane_normal=tuple(self.plane_normal),
            resolution=self.resolution,
            scale=self.scale
        )
        
        # Convert to poses with fixed orientation
        poses = []
        
        # Set tool orientation perpendicular to plane (assuming Z is normal)
        # For a vertical plane, orientation would need adjustment
        orientation = (np.pi, 0, 0) if abs(self.plane_normal[2]) < 0.001 else (0, np.pi, 0)
        
        # Add approach pose before each shape
        for point in trajectory_points:
            # Calculate approach point
            approach_point = (
                point[0] - self.approach_height * self.plane_normal[0],
                point[1] - self.approach_height * self.plane_normal[1],
                point[2] - self.approach_height * self.plane_normal[2]
            )
            
            # Add approach pose
            poses.append(create_pose(approach_point, orientation))
            
            # Add tracing pose
            poses.append(create_pose(point, orientation))
            
            # Add retract pose
            poses.append(create_pose(approach_point, orientation))
        
        return poses
    
    def trace_shapes(self):
        """Main function to trace all shapes"""
        
        self.get_logger().info('Starting shape tracing...')
        
        try:
            # Generate poses
            poses = self.generate_poses()
            self.get_logger().info(f'Generated {len(poses)} pose waypoints')
            
            # Plan trajectory
            waypoints = self.trajectory_planner.plan_cartesian_path(poses)
            self.get_logger().info(f'Planned {len(waypoints)} joint waypoints')
            
            # Create trajectory message
            trajectory = self.trajectory_planner.create_trajectory(waypoints)
            
            # Execute trajectory
            self.trajectory_planner.execute_trajectory(trajectory)
            
            self.get_logger().info('Shape tracing completed!')
            
        except Exception as e:
            self.get_logger().error(f'Error during shape tracing: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = ShapeTracerNode()
    
    try:
        node.trace_shapes()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import numpy as np
from typing import List, Tuple, Dict, Any
from enum import Enum

class ShapeType(Enum):
    LINE = "line"
    RECTANGLE = "rectangle"
    CIRCLE = "circle"
    POLYGON = "polygon"

class ShapeConverter:
    """
    Converts 2D shapes to 3D trajectories
    """
    
    def __init__(self):
        self.shapes = []
        
    def add_shape(self, shape_type: ShapeType, points: List[Tuple[float, float]], 
                  **kwargs):
        """
        Add a shape to be traced
        
        Args:
            shape_type: Type of shape
            points: 2D points defining the shape
            kwargs: Additional parameters (radius, etc.)
        """
        self.shapes.append({
            'type': shape_type,
            'points': points,
            'params': kwargs
        })
    
    def generate_trajectory(self, plane_origin: Tuple[float, float, float],
                           plane_normal: Tuple[float, float, float],
                           resolution: float = 0.01,
                           scale: float = 1.0) -> List[Tuple[float, float, float]]:
        """
        Generate 3D trajectory points for all shapes
        """
        trajectory = []
        
        for shape in self.shapes:
            shape_traj = self._generate_shape_trajectory(
                shape, plane_origin, plane_normal, resolution, scale
            )
            trajectory.extend(shape_traj)
            
            # Add transition between shapes
            if trajectory:
                trajectory.append(trajectory[-1])  # Hold position briefly
        
        return trajectory
    
    def _generate_shape_trajectory(self, shape: Dict[str, Any],
                                  plane_origin: Tuple[float, float, float],
                                  plane_normal: Tuple[float, float, float],
                                  resolution: float,
                                  scale: float) -> List[Tuple[float, float, float]]:
        """
        Generate trajectory for a single shape
        """
        from .utils import transform_point
        
        shape_type = shape['type']
        points = shape['points']
        
        if shape_type == ShapeType.LINE:
            return self._generate_line(points, plane_origin, plane_normal, resolution, scale)
        elif shape_type == ShapeType.RECTANGLE:
            return self._generate_rectangle(points, plane_origin, plane_normal, resolution, scale)
        elif shape_type == ShapeType.CIRCLE:
            return self._generate_circle(points, shape['params'].get('radius', 0.1),
                                       plane_origin, plane_normal, resolution, scale)
        elif shape_type == ShapeType.POLYGON:
            return self._generate_polygon(points, plane_origin, plane_normal, resolution, scale)
        else:
            raise ValueError(f"Unknown shape type: {shape_type}")
    
    def _generate_line(self, points: List[Tuple[float, float]],
                      plane_origin: Tuple[float, float, float],
                      plane_normal: Tuple[float, float, float],
                      resolution: float,
                      scale: float) -> List[Tuple[float, float, float]]:
        """Generate line trajectory"""
        from .utils import interpolate_points
        
        start = np.array(points[0])
        end = np.array(points[1])
        distance = np.linalg.norm(end - start)
        steps = max(int(distance / resolution), 2)
        
        trajectory = []
        for point_2d in interpolate_points(start, end, steps):
            point_3d = transform_point(tuple(point_2d), plane_origin, plane_normal, scale)
            trajectory.append(point_3d)
        
        return trajectory
    
    def _generate_rectangle(self, points: List[Tuple[float, float]],
                           plane_origin: Tuple[float, float, float],
                           plane_normal: Tuple[float, float, float],
                           resolution: float,
                           scale: float) -> List[Tuple[float, float, float]]:
        """Generate rectangle trajectory"""
        if len(points) < 2:
            raise ValueError("Rectangle requires at least 2 points (bottom-left and top-right)")
        
        bl = np.array(points[0])  # bottom-left
        tr = np.array(points[1])  # top-right
        br = np.array([tr[0], bl[1]])  # bottom-right
        tl = np.array([bl[0], tr[1]])  # top-left
        
        trajectory = []
        for segment in [(bl, br), (br, tr), (tr, tl), (tl, bl)]:
            segment_points = self._generate_line(
                [tuple(segment[0]), tuple(segment[1])],
                plane_origin, plane_normal, resolution, scale
            )
            trajectory.extend(segment_points)
        
        return trajectory
    
    def _generate_circle(self, points: List[Tuple[float, float]],
                        radius: float,
                        plane_origin: Tuple[float, float, float],
                        plane_normal: Tuple[float, float, float],
                        resolution: float,
                        scale: float) -> List[Tuple[float, float, float]]:
        """Generate circle trajectory"""
        if len(points) < 1:
            raise ValueError("Circle requires center point")
        
        center = np.array(points[0])
        circumference = 2 * np.pi * radius * scale
        steps = max(int(circumference / resolution), 36)  # Minimum 36 points for smooth circle
        
        trajectory = []
        for i in range(steps + 1):
            angle = 2 * np.pi * i / steps
            point_2d = center + radius * np.array([np.cos(angle), np.sin(angle)])
            point_3d = transform_point(tuple(point_2d), plane_origin, plane_normal, scale)
            trajectory.append(point_3d)
        
        return trajectory
    
    def _generate_polygon(self, points: List[Tuple[float, float]],
                         plane_origin: Tuple[float, float, float],
                         plane_normal: Tuple[float, float, float],
                         resolution: float,
                         scale: float) -> List[Tuple[float, float, float]]:
        """Generate polygon trajectory"""
        if len(points) < 3:
            raise ValueError("Polygon requires at least 3 points")
        
        trajectory = []
        for i in range(len(points)):
            start = np.array(points[i])
            end = np.array(points[(i + 1) % len(points)])
            segment_points = self._generate_line(
                [tuple(start), tuple(end)],
                plane_origin, plane_normal, resolution, scale
            )
            trajectory.extend(segment_points)
        
        return trajectory
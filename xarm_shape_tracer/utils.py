import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion
from tf_transformations import quaternion_from_euler
from typing import List, Tuple

def create_pose(position: Tuple[float, float, float], 
                orientation: Tuple[float, float, float] = (0, 0, 0)) -> Pose:
    """
    Create a Pose message from position and orientation (in radians)
    """
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    
    quat = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    
    return pose

def interpolate_points(start: np.ndarray, end: np.ndarray, steps: int) -> List[np.ndarray]:
    """
    Linear interpolation between two points
    """
    return [start + (end - start) * t for t in np.linspace(0, 1, steps)]

def transform_point(point_2d: Tuple[float, float], 
                    plane_origin: Tuple[float, float, float],
                    plane_normal: Tuple[float, float, float],
                    scale: float = 1.0) -> Tuple[float, float, float]:
    """
    Transform 2D point to 3D point on a plane
    """
    # Create basis vectors for the plane
    if abs(plane_normal[2]) < 0.999:
        # Plane is not horizontal
        x_axis = np.cross([0, 0, 1], plane_normal)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(plane_normal, x_axis)
    else:
        # Horizontal plane
        x_axis = np.array([1, 0, 0])
        y_axis = np.array([0, 1, 0])
    
    # Transform 2D point to 3D
    point_3d = np.array(plane_origin) + scale * (point_2d[0] * x_axis + point_2d[1] * y_axis)
    return tuple(point_3d)
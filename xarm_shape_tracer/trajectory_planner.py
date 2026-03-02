import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, Constraints, PositionConstraint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time
from typing import List, Tuple

class TrajectoryPlanner(Node):
    """
    Plans and executes trajectories for the xArm7
    """
    
    def __init__(self):
        super().__init__('trajectory_planner')
        
        # Parameters
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('goal_tolerance', 0.01)
        self.declare_parameter('joint_speed', 0.5)  # rad/s
        self.declare_parameter('max_acceleration', 0.5)  # rad/s^2
        
        # MoveIt! IK service
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik')
        
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, '/xarm/xarm7_trajectory_controller/joint_trajectory', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.current_joint_state = None
        
        # Joint names for xArm7
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'joint5', 'joint6', 'joint7'
        ]
    
    def joint_state_callback(self, msg):
        """Store current joint states"""
        self.current_joint_state = msg
    
    def compute_ik(self, target_pose: Pose) -> List[float]:
        """
        Compute inverse kinematics for target pose
        """
        if self.current_joint_state is None:
            raise RuntimeError("No joint state available")
        
        # Create IK request
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'xarm7'
        request.ik_request.pose_stamped.header.frame_id = 'world'
        request.ik_request.pose_stamped.pose = target_pose
        request.ik_request.timeout.sec = 5
        
        # Set robot state
        robot_state = RobotState()
        robot_state.joint_state = self.current_joint_state
        request.ik_request.robot_state = robot_state
        
        # Set constraints
        constraints = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'world'
        pos_constraint.link_name = 'eef'
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        pos_constraint.weight = 1.0
        constraints.position_constraints.append(pos_constraint)
        request.ik_request.constraints = constraints
        
        # Call IK service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().error_code.val == 1:
            return list(future.result().solution.joint_state.position)
        else:
            raise RuntimeError("IK solution not found")
    
    def create_trajectory(self, waypoints: List[List[float]], 
                         time_from_start: float = None) -> JointTrajectory:
        """
        Create a joint trajectory from waypoints
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        if time_from_start is None:
            time_from_start = len(waypoints) * 0.1
        
        for i, joints in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = joints
            point.time_from_start.sec = int(i * time_from_start / len(waypoints))
            point.time_from_start.nanosec = int((i * time_from_start / len(waypoints) - 
                                                point.time_from_start.sec) * 1e9)
            trajectory.points.append(point)
        
        return trajectory
    
    def plan_cartesian_path(self, poses: List[Pose]) -> List[List[float]]:
        """
        Plan a Cartesian path through the given poses
        """
        waypoints = []
        
        for pose in poses:
            try:
                joints = self.compute_ik(pose)
                waypoints.append(joints)
            except RuntimeError as e:
                self.get_logger().warn(f"Failed IK for pose: {e}")
                # Use previous joint position if available
                if waypoints:
                    waypoints.append(waypoints[-1])
                else:
                    raise
        
        return waypoints
    
    def execute_trajectory(self, trajectory: JointTrajectory):
        """
        Execute the trajectory
        """
        self.trajectory_pub.publish(trajectory)
        self.get_logger().info('Trajectory published')
        
        # Wait for execution (simple wait based on trajectory duration)
        if trajectory.points:
            duration = trajectory.points[-1].time_from_start.sec + \
                      trajectory.points[-1].time_from_start.nanosec * 1e-9
            time.sleep(duration + 1.0)
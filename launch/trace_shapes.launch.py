from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Start xArm7 simulation (if using Gazebo)
        ExecuteProcess(
            cmd=['ros2', 'launch', 'xarm_gazebo', 'xarm7_beside_table.launch.py'],
            output='screen'
        ),
        
        # Start MoveIt! for motion planning
        ExecuteProcess(
            cmd=['ros2', 'launch', 'xarm_moveit_config', 'xarm7_moveit.launch.py'],
            output='screen'
        ),
        
        # Start shape tracer node
        Node(
            package='xarm_shape_tracer',
            executable='shape_tracer',
            name='shape_tracer_node',
            output='screen',
            parameters=[os.path.join(
                os.path.dirname(__file__), '..', 'config', 'xarm_params.yaml'
            )]
        ),
        
        # Optional: RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                os.path.dirname(__file__), '..', 'config', 'shape_tracer.rviz'
            )],
            output='screen'
        )
    ])
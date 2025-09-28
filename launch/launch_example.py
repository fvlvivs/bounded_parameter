from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
	params_file = os.path.join(
		os.path.dirname(__file__), '..', 'params', 'example.yaml')
	return LaunchDescription([
		Node(
			package='bounded_parameter',
			executable='example_node',
			name='example_node',
			output='screen',
			parameters=[params_file]
		)
	])

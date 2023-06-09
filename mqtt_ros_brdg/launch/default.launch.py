from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

with open('src/mqtt-ros-bridge/conf.yaml', 'r') as f:
    config = yaml.safe_load(f.read())

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mqtt_bridge',
            executable='bridge_node',
            name='mqtt_bridge_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'broker_ip': config['broker_ip']},
                {'mqtt_cmd_vel_topic': config['mqtt_cmd_vel_topic']},
                {'mqtt_odom_topic': config['mqtt_odom_topic']},
                {'ros_cmd_vel_topic': config['ros_cmd_vel_topic']},
                {'ros_odom_topic': config['ros_odom_topic']}
            ]
        )
    ])
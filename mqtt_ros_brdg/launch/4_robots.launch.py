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
                {'mqtt_cmd_vel_topic': '/r0/cmd_vel'},
                {'mqtt_odom_topic': '/r0/odom'},
                {'ros_cmd_vel_topic': '/r0/cmd_vel'},
                {'ros_odom_topic': '/r0/odom'}
            ]
        ),

        Node(
            package='mqtt_bridge',
            executable='bridge_node',
            name='mqtt_bridge_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'broker_ip': config['broker_ip']},
                {'mqtt_cmd_vel_topic': '/r1/cmd_vel'},
                {'mqtt_odom_topic': '/r1/odom'},
                {'ros_cmd_vel_topic': '/r1/cmd_vel'},
                {'ros_odom_topic': '/r1/odom'}
            ]
        ),

        Node(
            package='mqtt_bridge',
            executable='bridge_node',
            name='mqtt_bridge_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'broker_ip': config['broker_ip']},
                {'mqtt_cmd_vel_topic': '/r2/cmd_vel'},
                {'mqtt_odom_topic': '/r2/odom'},
                {'ros_cmd_vel_topic': '/r2/cmd_vel'},
                {'ros_odom_topic': '/r2/odom'}
            ]
        ),

        Node(
            package='mqtt_bridge',
            executable='bridge_node',
            name='mqtt_bridge_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'broker_ip': config['broker_ip']},
                {'mqtt_cmd_vel_topic': '/r3/cmd_vel'},
                {'mqtt_odom_topic': '/r3/odom'},
                {'ros_cmd_vel_topic': '/r3/cmd_vel'},
                {'ros_odom_topic': '/r3/odom'}
            ]
        )
    ])
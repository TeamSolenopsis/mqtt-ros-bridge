import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odom
import json
import os
import math

class Bridge(Node):
    def __init__(self):
        super().__init__('bridge_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('broker_ip', rclpy.Parameter.Type.STRING),
                ('mqtt_cmd_vel_topic', rclpy.Parameter.Type.STRING),
                ('mqtt_odom_topic', rclpy.Parameter.Type.STRING),
                ('ros_cmd_vel_topic', rclpy.Parameter.Type.STRING),
                ('ros_odom_topic', rclpy.Parameter.Type.STRING)
            ]
        )

        self.broker_ip = self.get_parameter('broker_ip').get_parameter_value().string_value
        self.mqtt_cmd_vel_topic = self.get_parameter('mqtt_cmd_vel_topic').get_parameter_value().string_value
        self.mqtt_odom_topic  = self.get_parameter('mqtt_odom_topic').get_parameter_value().string_value
        self.ros_cmd_vel_topic = self.get_parameter('ros_cmd_vel_topic').get_parameter_value().string_value
        self.ros_odom_topic = self.get_parameter('ros_odom_topic').get_parameter_value().string_value

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(self.broker_ip, 1883, 60)
        self.client.loop_start()

        self.cmd_vel_publisher = self.create_publisher(Twist, self.ros_cmd_vel_topic, 10)
        self.odom_subscription = self.create_subscription(Odom, self.ros_odom_topic, self.odom_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)        

        self.create_timer(0.5, self.check_connection_mqtt)


    def check_connection_mqtt(self):
        if os.system(f'timeout 0.5 ping -c 1 {self.broker_ip}') != 0:
            null_cmd_vel = Twist()
            self.cmd_vel_publisher.publish(null_cmd_vel)
        

    def on_connect(self, client, userdata, flags, rc):
        self.client.subscribe(self.mqtt_cmd_vel_topic)

    def on_message(self, client, userdata, msg):
        twist = Twist()
        json_msg = json.loads(msg.payload.decode('utf-8'))
        twist.linear.x = float(json_msg['linear'])
        twist.angular.z = float(json_msg['angular'])
        self.cmd_vel_publisher.publish(twist)

    def odom_callback(self, msg: Odom):
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y

        w, x, y, z = msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = round(math.atan2(siny_cosp, cosy_cosp),2)

        odom = {
            "position": {
                "x": pose_x,
                "y": pose_y
            },
            "orientation": {
                "w": yaw
            }
        }

        self.client.publish(self.mqtt_odom_topic, json.dumps(odom))

def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

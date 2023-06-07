import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odom
import json

class Bridge(Node):
    def __init__(self):
        super().__init__('bridge_node')

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect('192.168.0.101', 1883, 60)
        self.client.loop_start()

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odom, 'odom', self.odom_callback, 10)        

    def on_connect(self, client, userdata, flags, rc):
        self.client.subscribe('cmd_vel')

    def on_message(self, client, userdata, msg):
        twist = Twist()
        json_msg = json.loads(msg.payload.decode('utf-8'))
        twist.linear.x = json_msg['linear']
        twist.angular.z = json_msg['angular']
        self.cmd_vel_publisher.publish(twist)

    def odom_callback(self, msg: Odom):
        self.client.publish('odom', '{{"x": {}, "y": {}, "theta": {}}}'.format(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z))

def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

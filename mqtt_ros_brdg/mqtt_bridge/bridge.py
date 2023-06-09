import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as Odom
import json

class Bridge(Node):
    def __init__(self):
        super().__init__('bridge_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('broker_ip', rclpy.Parameter.Type.STRING),
                ('mqtt_cmd_vel_topic', rclpy.Parameter.Type.STRING),
                ('mqtt_odom_topic', rclpy.Parameter.Type.STRING)
            ]
        )

        self.broker_ip = self.get_parameter('broker_ip').get_parameter_value().string_value
        self.mqtt_cmd_vel_topic = self.get_parameter('mqtt_cmd_vel_topic').get_parameter_value().string_value
        self.mqtt_odom_topic  = self.get_parameter('mqtt_odom_topic').get_parameter_value().string_value

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_disconnect = self.on_disconnect

        self.client.connect(self.broker_ip, 1883, 60)
        self.client.loop_start()

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odom, 'odom', self.odom_callback, qos_profile=rclpy.qos.qos_profile_sensor_data)        

        self.create_timer(0.5, self.send_cmd_vel)
        self.current_cmd_vel = Twist()

    def send_cmd_vel(self):
        self.cmd_vel_publisher.publish(self.current_cmd_vel)

    def on_connect(self, client, userdata, flags, rc):
        self.client.subscribe(self.mqtt_cmd_vel_topic)

    def on_disconnect(self, client, userdata, rc):
        print('disconnected')
        self.current_cmd_vel = Twist()
        self.cmd_vel_publisher.publish(Twist())
        self.client.reconnect()

    def on_message(self, client, userdata, msg):
        twist = Twist()
        json_msg = json.loads(msg.payload.decode('utf-8'))
        twist.linear.x = float(json_msg['linear'])
        twist.angular.z = float(json_msg['angular'])
        self.current_cmd_vel = twist
        self.cmd_vel_publisher.publish(twist)

    def odom_callback(self, msg: Odom):
        odom = {
            "position": {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y
            },
            "orientation": {
                "w": msg.pose.pose.orientation.w,
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z
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

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry as Odom
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
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
                ('mqtt_assembler_pose_topic', rclpy.Parameter.Type.STRING),
                ('mqtt_assembler_arrived_topic', rclpy.Parameter.Type.STRING),
                ('ros_cmd_vel_topic', rclpy.Parameter.Type.STRING),
                ('ros_odom_topic', rclpy.Parameter.Type.STRING)
            ]
        )

        self.broker_ip = self.get_parameter('broker_ip').get_parameter_value().string_value
        self.mqtt_cmd_vel_topic = self.get_parameter('mqtt_cmd_vel_topic').get_parameter_value().string_value
        self.mqtt_odom_topic  = self.get_parameter('mqtt_odom_topic').get_parameter_value().string_value
        self.mqtt_assembler_pose_topic  = self.get_parameter('mqtt_assembler_pose_topic').get_parameter_value().string_value
        self.mqtt_assembler_arrived_topic = self.get_parameter('mqtt_assembler_arrived_topic').get_parameter_value().string_value
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
        self.create_timer(0.5, self.send_cmd_vel)
        self.current_cmd_vel = Twist()

        self.action_navigate_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_cmd_vel(self):
        self.cmd_vel_publisher.publish(self.current_cmd_vel)

    def check_connection_mqtt(self):
        if os.system(f'timeout 0.5 ping -c 1 {self.broker_ip}') != 0:
            self.current_cmd_vel = Twist()
            self.cmd_vel_publisher.publish(self.current_cmd_vel)
        

    def on_connect(self, client, userdata, flags, rc):
        self.client.subscribe(self.mqtt_cmd_vel_topic)

    def on_message(self, client, userdata, msg):
        if msg.topic == self.mqtt_cmd_vel_topic:
            twist = Twist()
            json_msg = json.loads(msg.payload.decode('utf-8'))
            twist.linear.x = float(json_msg['linear'])
            twist.angular.z = float(json_msg['angular'])
            self.current_cmd_vel = twist
            self.cmd_vel_publisher.publish(twist)
        elif msg.topic == self.mqtt_assembler_pose_topic:
            self.pose_topic_callback(msg) 

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

    def pose_topic_callback(self, msg):
        goal_pose = PoseStamped()
        json_msg = json.loads(msg.payload.decode('utf-8'))
        goal_pose.pose.position.x = float(json_msg['position']['x'])
        goal_pose.pose.position.y = float(json_msg['position']['y'])
        goal_pose.pose.orientation.w = float(json_msg['orientation']['w'])

        goal_msg = NavigateToPose.Goal()
        goal_msg.goal = goal_pose

        self.action_navigate_pose_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg)

        # Handle the response
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Create a callback to handle the result
        goal_handle.add_done_callback(self.result_callback)

    def result_callback(self, future):
        goal_handle = future.result()
        status = goal_handle.status
        result = goal_handle.result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded')
            # Process the result here if needed
            self.client.publish(self.mqtt_assembler_arrived_topic, json.dumps("arrived"))
        else:
            self.get_logger().info('Goal failed')
            # Handle the failure case here
 
def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

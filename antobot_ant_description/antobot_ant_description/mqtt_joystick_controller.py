import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import json

class JoystickMQTTController(Node):
    def __init__(self):
        super().__init__('joystick_mqtt_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect("localhost", 1883, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe("antobot/joystick")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            x = -float(data.get("y", 0.0))  # linear speed
            y = -float(data.get("x", 0.0))  # angular speed

            twist = Twist()
            twist.linear.x = x * 0.5       # scale down if needed
            twist.angular.z = y * 0.5

            self.publisher_.publish(twist)
        except Exception as e:
            self.get_logger().error(f"Error parsing message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickMQTTController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

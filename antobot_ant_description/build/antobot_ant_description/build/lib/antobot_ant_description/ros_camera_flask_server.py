#!/usr/bin/env python3
import threading

from flask import Flask, Response
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

app = Flask(__name__)

# ——— Globals ———
bridge = CvBridge()
front_frame = None
rear_frame = None
frame_lock = threading.Lock()

current_velocity = 0.0
twist_lock = threading.Lock()

CAPTURE_WIDTH = 640
CAPTURE_HEIGHT = 360

# ——— ROS 2 node to subscribe to cameras and cmd_vel ———
class CameraBridge(Node):
    def __init__(self):
        super().__init__('ros_camera_flask_bridge')
        # Front camera subscription
        self.create_subscription(
            Image, '/front_camera/image_raw',
            self.front_cb, 10)
        # Rear camera subscription
        self.create_subscription(
            Image, '/rear_camera/image_raw',
            self.rear_cb, 10)
        # Velocity subscription
        self.create_subscription(
            Twist, '/cmd_vel',
            self.twist_cb, 10)

    def front_cb(self, msg: Image):
        global front_frame
        try:
            frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.resize(frame, (CAPTURE_WIDTH, CAPTURE_HEIGHT))
            with frame_lock:
                front_frame = frame
        except Exception as e:
            self.get_logger().error(f"Front camera CVBridge error: {e}")

    def rear_cb(self, msg: Image):
        global rear_frame
        try:
            frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = cv2.resize(frame, (CAPTURE_WIDTH, CAPTURE_HEIGHT))
            with frame_lock:
                rear_frame = frame
        except Exception as e:
            self.get_logger().error(f"Rear camera CVBridge error: {e}")

    def twist_cb(self, msg: Twist):
        global current_velocity
        with twist_lock:
            current_velocity = msg.linear.x

def start_ros_node():
    rclpy.init()
    node = CameraBridge()
    rclpy.spin(node)
    rclpy.shutdown()

# ——— MJPEG stream endpoint ———
@app.route('/')
def index():
    return (
        "<h1>ROS Camera MJPEG Stream</h1>"
        "<p>Front camera when moving ≥0, rear when moving &lt;0</p>"
        "<p>View feed at <a href='/video'>/video</a></p>"
    )

@app.route('/video')
def video_feed():
    def generate():
        boundary = b'--frame\r\n'
        # default center cropping removed; full frames used
        while True:
            # Choose which frame to serve
            with twist_lock:
                vel = current_velocity
            with frame_lock:
                frame = front_frame.copy() if vel >= 0 else rear_frame.copy() \
                        if vel < 0 else None

            if frame is None:
                continue

            ok, buf = cv2.imencode('.jpg', frame)
            if not ok:
                continue

            yield (
                boundary +
                b'Content-Type: image/jpeg\r\n\r\n' +
                buf.tobytes() +
                b'\r\n'
            )
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

def main():
    """Entry point for ros2 run antobot_ant_description ros_camera_flask"""
    # Start ROS node in background
    ros_thread = threading.Thread(target=start_ros_node, daemon=True)
    ros_thread.start()
    # Start MJPEG server
    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == '__main__':
    main()

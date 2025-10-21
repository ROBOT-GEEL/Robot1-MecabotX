#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloPeopleNode(Node):
    def __init__(self):
        super().__init__('yolo_people_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('use_cuda', True)
        self.declare_parameter('publish_window', False)  # show cv2.imshow

        self.image_topic = self.get_parameter('image_topic').value
        self.conf = float(self.get_parameter('conf').value)
        self.use_cuda = bool(self.get_parameter('use_cuda').value)
        self.publish_window = bool(self.get_parameter('publish_window').value)

        # Model
        self.model = YOLO('yolov8n.pt')
        if self.use_cuda:
            try:
                self.model.to('cuda')
                self.get_logger().info('Using CUDA')
            except Exception as e:
                self.get_logger().warn(f'CUDA unavailable: {e}')

        # I/O
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )
        self.pub_annot = self.create_publisher(Image, '/detected_image', 10)

        self.get_logger().info(f'Listening on {self.image_topic}')
        self.get_logger().info('Publishing annotated image on /detected_image')

    def on_image(self, msg: Image):
        # ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO inference (people only = class 0)
        results = self.model.predict(
            source=frame, classes=[0], conf=self.conf, verbose=False
        )
        annotated = results[0].plot()

        # Optional on-screen window (skip if running headless/SSH)
        if self.publish_window:
            cv2.imshow('YOLO People (ROS topic)', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

        # Publish annotated image to ROS so RViz2 can show it
        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = msg.header  # preserve timestamp/frame_id
        self.pub_annot.publish(out)

def main():
    rclpy.init()
    node = YoloPeopleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

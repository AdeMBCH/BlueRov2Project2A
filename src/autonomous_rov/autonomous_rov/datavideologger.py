import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class DataAndVideoLogger(Node):
    def __init__(self):
        super().__init__('data_and_video_logger')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/bluerov2/camera/image',
            self.image_callback,
            10)
        self.point_sub = self.create_subscription(
            Float64MultiArray,
            '/bluerov2/tracked_point',
            self.point_callback,
            10)
        self.segment_sub = self.create_subscription(
            Float64MultiArray,
            '/bluerov2/tracked_segment',
            self.segment_callback,
            10)
        
        self.points = []
        self.segments = []
        self.video_writer = None

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.video_writer is None:
            self.video_writer = cv2.VideoWriter('log_video.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30.0, (cv_image.shape[1], cv_image.shape[0]))
        self.video_writer.write(cv_image)

    def point_callback(self, msg):
        self.points.append(msg.data)
        self.get_logger().info(f'Received point: {msg.data}')

    def segment_callback(self, msg):
        self.segments.append(msg.data)
        self.get_logger().info(f'Received segment: {msg.data}')

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
        np.save('points.npy', np.array(self.points))
        np.save('segments.npy', np.array(self.segments))
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataAndVideoLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

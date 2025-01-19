#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CompressedImage  # need to import CompressedImage to fit the topic used by bags
import numpy as np
import cv2
from . import camera_parameters as cam
from cv_bridge import CvBridge
import torch

import warnings
warnings.filterwarnings("ignore", category=FutureWarning)




class ImageProcessingYoloNode(Node):
    def __init__(self):

        # camera parameters
        self.u0 = 320
        self.v0 = 240
        self.lx = 455
        self.ly = 455
        self.kud = 0.00683
        self.kdu = -0.01424

        super().__init__('image_processing_yolo_node')

        # self.pub_tracked_point = self.create_publisher(Float64MultiArray, 'tracked_point', 10)
        # self.pub_desired_point = self.create_publisher(Float64MultiArray, 'desired_point', 10)
        self.detection_publisher = self.create_publisher(Float64MultiArray, 'detected_buoys', 10)


        self.subscription = self.create_subscription(
            CompressedImage,
            '/br4/raspicam_node/image/compressed',
            self.cameracallback,
            1)

        self.bridge = CvBridge()
        self.get_logger().info("Buoy detection node initialized.")

        # YOLOv5 Model

        self.model = torch.hub.load('/home/projet_sysmer/ros2_ws/src/yolov5',
                                    'custom',
                                    path='/home/projet_sysmer/ros2_ws/src/yolov5/runs/train/exp6/weights/best.pt',
                                    source='local')

        self.model.conf = 0.5  # Seuil de confiance pour les détections à modifier en f des resultats
        self.get_logger().info("pre-trained YOLOv5 model loaded successfully.")


    def cameracallback(self, msg):
        self.get_logger().info("Image received for processing.")

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        results = self.model(frame)
        detections = results.xyxy[0] # resultat des detections

        detected_buoys = []
        for det in detections:
            x1, y1, x2, y2, conf, cls = det.tolist()
            self.get_logger().info(cls)
            if conf > self.model.conf :
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                detected_buoys.append([cx, cy])

                cv2.rectangle(frame,
                              (int(x1), int(y1)),
                              (int(x2), int(y2)),
                              (0, 255, 0),
                              2)

                cv2.putText(frame,
                            f"Class: {int(cls)}, Conf: {conf:.2f}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2)

        if detected_buoys:

            buoy_positions = Float64MultiArray(data=np.array(detected_buoys).flatten())
            self.detection_publisher.publish(buoy_positions)

        cv2.imshow("YOLOv5 Buoy Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



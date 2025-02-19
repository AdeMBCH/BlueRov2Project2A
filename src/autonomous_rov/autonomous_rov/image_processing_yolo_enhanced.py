#!/usr/bin/env python3
import csv
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CompressedImage   # need to import CompressedImage to fit the topic used by bags
from mavros_msgs.msg import CameraImageCaptured
import numpy as np
import cv2
from cv_bridge import CvBridge
import torch
import time



cv2.namedWindow('Result')

# Clicks for feedback
set_desired_point = False
get_hsv = False
mouseX, mouseY = 0, 0

# camera parameters
u0 = 320
v0 = 240
lx = 455
ly = 455
kud = 0.00683
kdu = -0.01424



def overlay_box(image, box, label, conf, color=(0, 255, 0)):
    """
    Dessine un rectangle et affiche l'étiquette avec la confiance sur l'image.
    """
    x1, y1, x2, y2 = [int(v) for v in box]
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
    cv2.putText(image, f"{label} {conf:.2f}", (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def click_detect(event, x, y, flags, param):
    global set_desired_point, mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        mouseX, mouseY = x, y
    if event == cv2.EVENT_RBUTTONDOWN:
        set_desired_point = True
        mouseX, mouseY = x, y

# convert a pixel coordinate to meters using defaut calibration parameters
def convertOnePoint2meter(pt):
    global u0, v0, lx, ly
    return (float(pt[0]) - u0) / lx, (float(pt[1]) - v0) / ly


def overlay_points(image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
    cv2.circle(image, (int(pt[0]), int(pt[1])), int(4 * scale + 1), (b, g, r), -1)
    position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)


class ImageProcessingWithYolo(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        self.pub_tracked_point = self.create_publisher(Float64MultiArray, 'tracked_point', 10)
        self.pub_desired_point = self.create_publisher(Float64MultiArray, 'desired_point', 10)
        self.pub_tracked_segment = self.create_publisher(Float64MultiArray, 'tracked_segment', 10)

        # Chargement du modèle YOLOv5 custom
        self.get_logger().info("Chargement du modèle YOLOv5...")
        self.model = torch.hub.load('/home/projet_sysmer/ros2_ws/src/yolov5',
                                    'custom',
                                    path='/home/projet_sysmer/ros2_ws/src/yolov5/runs/train/exp3/weights/best.pt',
                                    source='local')
        self.model.conf = 0.60  # Seuil de confiance
        self.get_logger().info("Modèle chargé avec succès.")

        self.bridge = CvBridge()  # CvBridge for converting ROS images to OpenCV format
        self.image_sub = self.create_subscription(
            Image,
            '/bluerov2/camera/image',
            self.image_callback,
            10)


        self.last_tracked_point = None  # en mètres
        self.last_time = None
        self.max_speed = 1.5  # seuil de vitesse en m/s (à ajuster)

        cv2.namedWindow("Result")
        cv2.setMouseCallback("Result", click_detect)

    def image_callback(self, msg):
        self.get_logger().info('Received new frame!')
        start_time = time.time()

        # Conversion de l'image ROS vers OpenCV
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Pour les images compressées:
            # image = self.bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image: {e}")
            return

        if image is None:
            self.get_logger().error("L'image convertie est None")
            return

        image_height, image_width, _ = image.shape

        desired_point = [image_width / 2, image_height / 2]

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        results = self.model(image_rgb)
        detections = results.xyxy[0].cpu().numpy()

        valid_update = True
        tracked_point = None
        segment_width = None
        segment_y = None

        if detections.shape[0] > 0:
            x1, y1, x2, y2, conf, cls = detections[0]

            overlay_box(image, [x1, y1, x2, y2], self.model.names[int(cls)], conf)

            segment_y = int((y1 + y2) // 2)

            left_point = (int(x1), segment_y)
            right_point = (int(x2), segment_y)
            cv2.line(image, left_point, right_point, (0, 0, 0), 5)

            # Conversion en mètres
            left_point_meter = convertOnePoint2meter(left_point)
            right_point_meter = convertOnePoint2meter(right_point)
            dx = left_point_meter[0] - right_point_meter[0]
            dy = left_point_meter[1] - right_point_meter[1]
            current_width = np.sqrt(dx ** 2 + dy ** 2) * 0.95  # ajustement éventuel
            if current_width >= 0.07:
                segment_width = current_width
            else:
                self.get_logger().info(f"Segment width too small: {current_width:.4f}")

            # Calculer le point suivi comme le centre de la bounding box et l'aligner verticalement
            cx = int((x1 + x2) // 2)
            tracked_point = [cx, segment_y]

            # Conversion en mètres du point suivi
            current_point_meter = convertOnePoint2meter(tracked_point)

            # Calcul de la vitesse du point suivi
            current_time = time.time()
            if self.last_tracked_point is not None and self.last_time is not None:
                dt = current_time - self.last_time
                if dt > 0:
                    displacement = np.linalg.norm(np.array(current_point_meter) - np.array(self.last_tracked_point))
                    speed = displacement / dt
                    if speed > self.max_speed:
                        self.get_logger().info(f"Vitesse trop élevée: {speed:.2f} m/s, mise à jour ignorée")
                        valid_update = False
                        current_point_meter = self.last_tracked_point
                    else:
                        self.last_tracked_point = current_point_meter
                        self.last_time = current_time
                else:
                    self.last_tracked_point = current_point_meter
                    self.last_time = current_time
            else:
                self.last_tracked_point = current_point_meter
                self.last_time = current_time

        else:
            self.get_logger().info("Aucune détection avec YOLO, aucune publication pour tracked_point et segment.")

        if tracked_point is not None and segment_width is not None and valid_update:
            overlay_points(image, tracked_point, 0, 255, 0, 'current tracked buoy')
            current_point_msg = Float64MultiArray(data=list(current_point_meter))
            self.pub_tracked_point.publish(current_point_msg)

            segment_msg = Float64MultiArray(data=[segment_width])
            self.pub_tracked_segment.publish(segment_msg)
        else:
            self.get_logger().info(
                "Point suivi ou segment manquant, ou vitesse trop élevée, aucune publication n'est effectuée.")

        overlay_points(image, desired_point, 255, 0, 0, 'desired point')
        desired_point_meter = convertOnePoint2meter(desired_point)
        desired_point_msg = Float64MultiArray(data=desired_point_meter)
        self.pub_desired_point.publish(desired_point_msg)

        cv2.imshow("Result", image)
        cv2.waitKey(2)

        end_time = time.time()
        inference_time = (end_time - start_time) * 1000
        self.get_logger().info(f"Inference time with YOLOv5: {inference_time:.2f} milliseconds")


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingWithYolo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CompressedImage   # need to import CompressedImage to fit the topic used by bags
from mavros_msgs.msg import CameraImageCaptured
import numpy as np
import cv2
from . import camera_parameters as cam
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
        self.model.conf = 0.70  # Seuil de confiance
        self.get_logger().info("Modèle chargé avec succès.")

        self.bridge = CvBridge()  # CvBridge for converting ROS images to OpenCV format
        # self.image_sub = self.create_subscription(
        #     Image,
        #     '/bluerov2/camera/image',
        #     self.image_callback,
        #     10
        # )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/br4/raspicam_node/image/compressed',
            self.image_callback,
            10)

        cv2.namedWindow("Result")
        cv2.setMouseCallback("Result", click_detect)

    def image_callback(self, msg):
        self.get_logger().info('Received new frame !')
        start_time = time.time()

        # image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = self.bridge.compressed_imgmsg_to_cv2(msg)

        global set_desired_point

        if image is None:
            self.get_logger().error(f"Erreur : Impossible de charger l'image à partir de {self.image}")
            return

        image_height, image_width, _ = image.shape

        desired_point = [image_width / 2, image_height / 2]

        if set_desired_point:
            desired_point = [mouseX, mouseY]
            set_desired_point = False

        # Conversion en RGB pour YOLOv5
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Mesurer le temps d'inférence

        results = self.model(image_rgb)


        # Récupérer les détections : [x1, y1, x2, y2, confiance, classe]
        detections = results.xyxy[0].cpu().numpy()
        self.get_logger().info("Détections :")
        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            self.get_logger().info(
                f"Classe: {int(cls)} ({self.model.names[int(cls)]}) - Confiance: {conf:.4f} - Coordonnées: {x1:.4f}, {y1:.4f}, {x2:.4f}, {y2:.4f}")
            overlay_box(image, [x1, y1, x2, y2], self.model.names[int(cls)], conf)

        if detections.shape[0]>0:
            x1, y1, x2, y2,_,_ = detections[0]

            right_point= (int(x1), int((y1 + y2)//2))
            left_point = (int(x2), int((y1 + y2) // 2))
            # self.get_logger().info(right_point)
            # # self.get_logger().info(type(right_point))
            # Afficher le segment
            cv2.line(image, left_point, right_point, (0, 0, 0), 5)

            # Convertir en mètres
            left_point_meter = convertOnePoint2meter(left_point)
            right_point_meter = convertOnePoint2meter(right_point)

            #publication du message
            segment_msg = Float64MultiArray(data=[*left_point_meter, *right_point_meter])
            self.pub_tracked_segment.publish(segment_msg)

            cx,cy = int((x1+x2)//2), int((y1 + y2)//2)
            cv2.circle(image,(int(cx),int(cy)),radius=5,color=(0,255,0))
            current_point = [cx, cy]
            overlay_points(image, current_point, 0, 255, 0, 'current tracked buoy')
            current_point_meter = convertOnePoint2meter(current_point)
            current_point_msg = Float64MultiArray(data=current_point_meter)
            self.pub_tracked_point.publish(current_point_msg)

        if desired_point is not None:
            overlay_points(image, desired_point, 255, 0, 0, 'desired point')
            desired_point_meter = convertOnePoint2meter(desired_point)
            desired_point_msg = Float64MultiArray(data=desired_point_meter)
            self.pub_desired_point.publish(desired_point_msg)

        # Afficher l'image annotée dans une fenêtre OpenCV
        cv2.imshow("Result", image)
        cv2.waitKey(2)

        end_time = time.time()
        inference_time = (end_time - start_time) * 1000  # en ms
        self.get_logger().info(f"Temps d'inférence : {inference_time:.2f} ms")




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

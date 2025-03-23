#!/usr/bin/env python3
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GLib, GdkPixbuf
import rclpy
import cv2
import time
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node
import subprocess

class BlueRovGUI:
    def __init__(self):
        self.builder = Gtk.Builder()
        self.builder.add_from_file(
            "/home/projet_sysmer/ros2_ws/src/autonomous_rov/autonomous_rov/BlueRov2GUISysmer2A.glade"
        )
        self.builder.connect_signals(self)

        self.window = self.builder.get_object("GtkWindow")
        self.raw_video_image = self.builder.get_object("raw_video_image")
        self.processed_video_image = self.builder.get_object("processed_video_image")
        self.tracked_video_image = self.builder.get_object("tracked_video_image")
        self.status_label = self.builder.get_object("status_label")

        self.window.connect("destroy", Gtk.main_quit)

    def update_image(self, widget, pixbuf):
        widget.set_from_pixbuf(pixbuf)

    def on_launch_button_clicked(self, widget):
        self.status_label.set_text("Status: Building...")
        
        # Commande principale (MAVROS) dans un nouveau terminal
        command = f"""
        terminator --working-directory=~/ros2_ws -e '

        colcon build --symlink-install --packages-select autonomous_rov ||

        source install/setup.bash

        # Lancement du fichier launch dans un nouveau terminal
        gnome-terminal --working-directory=~/ros2_ws -e "ros2 launch autonomous_rov run_mavros.launch"

        sleep 6

        gnome-terminal --working-directory=~/ros2_ws -e "ros2 launch autonomous_rov run_gamepad.launch"

        sleep 5

        gnome-terminal --working-directory=~/ros2_ws -e "ros2 launch autonomous_rov run_video.launch"

        sleep 3

        gnome-terminal --working-directory=~/ros2_ws -e "ros2 launch autonomous_rov run_listener_sysmer.launch"

        # Garde le terminal ouvert pour interagir
        exec bash
        '
        """
        subprocess.Popen(command, shell=True)



    def on_log_button_clicked(self, widget):
        print("Logging system information...")

class VideoSubscriber(Node):
    def __init__(self, gui):
        super().__init__('blue_rov_video_subscriber')
        self.gui = gui
        self.bridge = CvBridge()
        self.last_frame_time = time.time()
        self.frame_rate = 30  # Hz

        self.subscription = self.create_subscription(
            Image,
            '/bluerov2/camera/image',
            self.image_callback,
            10)

    def image_callback(self, msg):
        try:
            current_time = time.time()
            if (current_time - self.last_frame_time) > (1.0 / self.frame_rate):
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # Conversion BGR vers RGB
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                
                height, width = cv_image.shape[:2]
                pixbuf = GdkPixbuf.Pixbuf.new_from_data(
                    cv_image.tobytes(),
                    GdkPixbuf.Colorspace.RGB,
                    False,
                    8,
                    width,
                    height,
                    width * 3
                )
                
                GLib.idle_add(self.gui.update_image, self.gui.raw_video_image, pixbuf)
                self.last_frame_time = current_time

        except Exception as e:
            self.get_logger().error(f"Erreur de traitement d'image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    gui = BlueRovGUI()
    video_subscriber = VideoSubscriber(gui)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(video_subscriber)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    gui.window.show_all()
    Gtk.main()

    executor.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

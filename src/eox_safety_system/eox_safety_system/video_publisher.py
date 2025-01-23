#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class UsbCameraPublisher(Node):
    def __init__(self):
        super().__init__('usb_camera_publisher')

        # Publish the image on the raw image and info topics to visualize in rviz
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Set the fps
        self.timer = self.create_timer(0.05, self.timer_callback)  # Adjust frequency as needed (10 Hz)

        # Open the camera stream 
        self.cap = self.initialize_camera()

        # Link OpenCV to ROS
        self.bridge = CvBridge()

	# Display info to check that the node works
        self.get_logger().info("USB Camera Publisher Node initialized and running.")

    def initialize_camera(self):
        # Set the camera ID and specify resolutions
        cap = cv2.VideoCapture(0)  # Change device ID as needed
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)

        if not cap.isOpened():
            self.get_logger().error("Could not open video device")
            self.destroy_node()
            exit(1)

        self.get_logger().info("Camera successfully initialized.")
        return cap

    def timer_callback(self):
        """Capture and publish images and camera info periodically."""
        ret, frame = self.cap.read()

        if ret:
            # Publish the image
            ros_image = self.convert_to_ros_image(frame)
            self.image_publisher.publish(ros_image)

            # Publish the camera info
            camera_info = self.generate_camera_info(ros_image)
            self.camera_info_publisher.publish(camera_info)

            self.get_logger().info('Published image and camera info')
        else:
            self.get_logger().error("Failed to capture frame from camera")

    def convert_to_ros_image(self, frame):
        """Convert OpenCV frame to a ROS Image message."""
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_frame"
        return ros_image

    def generate_camera_info(self, ros_image):
        """Generate a CameraInfo message."""
        camera_info = CameraInfo()
        camera_info.header.stamp = ros_image.header.stamp
        camera_info.header.frame_id = "camera_frame"

        # Camera resolution
        camera_info.width = 640
        camera_info.height = 512

        # Intrinsic calibration parameters
        fx, fy = 4.3, 4.3  # Adjust focal lengths
        cx, cy = 480.0, 620.0    # Adjust optical centers
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return camera_info

    def __del__(self):
        """Release the camera when the node is destroyed."""
        if self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Released the camera resource.")

def main(args=None):
    rclpy.init(args=args)

    try:
        camera_publisher = UsbCameraPublisher()
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


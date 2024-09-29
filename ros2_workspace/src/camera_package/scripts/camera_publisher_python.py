#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import cv2
from cv_bridge import CvBridge
import yaml
import numpy as np
import os

class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        # Declare the parameters to accept paths to the left and right YAML config files
        self.declare_parameter('left_config_file', '')
        self.declare_parameter('right_config_file', '')

        # Retrieve the config file paths from the parameters
        left_config_file = self.get_parameter('left_config_file').get_parameter_value().string_value
        right_config_file = self.get_parameter('right_config_file').get_parameter_value().string_value

        # Check if both YAML config files are valid
        if not os.path.isfile(left_config_file):
            self.get_logger().error(f"Invalid left config file: {left_config_file}")
            rclpy.shutdown()
            return
        if not os.path.isfile(right_config_file):
            self.get_logger().error(f"Invalid right config file: {right_config_file}")
            rclpy.shutdown()
            return

        # Parse the configuration files for both cameras
        config_left = self.parse_yaml_config(left_config_file)
        config_right = self.parse_yaml_config(right_config_file)

        # Open video capture device using OpenCV
        self.cap = cv2.VideoCapture(0)  # Open /dev/video0
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open /dev/video0")
            rclpy.shutdown()
            return

        # Set video frame width and height (assuming symmetric resolution from the left camera config)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config_left['image_width'] * 2)  # Assuming side-by-side stereo
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config_left['image_height'])

        # Create publishers for raw and compressed images for both cameras
        self.pub_left = self.create_publisher(Image, 'stereo/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, 'stereo/right/image_raw', 10)
        self.pub_left_compressed = self.create_publisher(CompressedImage, 'stereo/left/image_raw/compressed', 10)
        self.pub_right_compressed = self.create_publisher(CompressedImage, 'stereo/right/image_raw/compressed', 10)

        # Create publishers for camera info (calibration data) for both cameras
        self.pub_left_camera_info = self.create_publisher(CameraInfo, 'stereo/left/camera_info', 10)
        self.pub_right_camera_info = self.create_publisher(CameraInfo, 'stereo/right/camera_info', 10)

        # Create publishers for rectified images
        #self.pub_left_rect = self.create_publisher(Image, 'stereo/left/image_rect', 10)
        #self.pub_right_rect = self.create_publisher(Image, 'stereo/right/image_rect', 10)

        # Timer to publish images at approximately 30 FPS
        self.timer = self.create_timer(0.033, self.publish_images)

        # Bridge to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        # Load the calibration data for both cameras from config
        self.left_camera_info = self.generate_camera_info(config_left)
        self.right_camera_info = self.generate_camera_info(config_right)

        # Initialize rectification maps
        self.init_rectification_maps(config_left, config_right)

    def parse_yaml_config(self, yaml_file):
        """
        Parse the camera configuration from the YAML file.
        """
        with open(yaml_file, 'r') as file:
            config = yaml.safe_load(file)

        # Extract necessary parameters
        image_width = config['image_width']
        image_height = config['image_height']
        camera_matrix = np.array(config['camera_matrix']['data']).reshape(
            (config['camera_matrix']['rows'], config['camera_matrix']['cols']))
        distortion = np.array(config['distortion_coefficients']['data'])
        rectification = np.array(config['rectification_matrix']['data']).reshape(
            (config['rectification_matrix']['rows'], config['rectification_matrix']['cols']))
        projection = np.array(config['projection_matrix']['data']).reshape(
            (config['projection_matrix']['rows'], config['projection_matrix']['cols']))
        distortion_model = config['distortion_model']

        return {
            'image_width': image_width,
            'image_height': image_height,
            'camera_matrix': camera_matrix,
            'distortion': distortion,
            'rectification': rectification,
            'projection': projection,
            'distortion_model': distortion_model
        }

    def generate_camera_info(self, config):
        """
        Generate a CameraInfo message using the calibration parameters from the YAML config.
        """
        camera_info = CameraInfo()
        camera_info.width = config['image_width']
        camera_info.height = config['image_height']
        camera_info.k = config['camera_matrix'].flatten().tolist()
        camera_info.d = config['distortion'].tolist()
        camera_info.distortion_model = config['distortion_model']
        camera_info.r = config['rectification'].flatten().tolist()
        camera_info.p = config['projection'].flatten().tolist()

        return camera_info

    def init_rectification_maps(self, config_left, config_right):
        """
        Initialize rectification maps for both cameras.
        """
        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            config_left['camera_matrix'], config_left['distortion'], 
            config_left['rectification'], config_left['projection'], 
            (config_left['image_width'], config_left['image_height']), cv2.CV_32FC1)

        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            config_right['camera_matrix'], config_right['distortion'], 
            config_right['rectification'], config_right['projection'], 
            (config_right['image_width'], config_right['image_height']), cv2.CV_32FC1)

    def publish_images(self):
        ret, frame = self.cap.read()

        if ret:
            # Split the frame into left and right images
            left_image = frame[:, :frame.shape[1] // 2]
            right_image = frame[:, frame.shape[1] // 2:]

            # Convert the images to grayscale
            # left_image_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            # right_image_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

            camera_clock = self.get_clock().now().to_msg()

            # Publish left image and camera info
            self.publish_image(left_image, self.pub_left, self.pub_left_compressed, 'camera_left_frame', encoding='bgr8', camera_clock = camera_clock)
            self.publish_camera_info(self.pub_left_camera_info, self.left_camera_info, camera_clock)

            # Publish right image and camera info
            self.publish_image(right_image, self.pub_right, self.pub_right_compressed, 'camera_right_frame', encoding='bgr8', camera_clock = camera_clock)
            self.publish_camera_info(self.pub_right_camera_info, self.right_camera_info, camera_clock)

            # Rectify images and publish
            # left_rectified = cv2.remap(left_image_gray, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
            # right_rectified = cv2.remap(right_image_gray, self.right_map1, self.right_map2, cv2.INTER_LINEAR)

            # self.publish_image(left_rectified, self.pub_left_rect, None, 'camera_left_rectified', encoding='mono8')
            # self.publish_image(right_rectified, self.pub_right_rect, None, 'camera_right_rectified', encoding='mono8')

    def publish_image(self, image, raw_pub, compressed_pub, frame_id, encoding='bgr8', camera_clock = 0):
        # Convert OpenCV image to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        image_msg.header.stamp = camera_clock
        image_msg.header.frame_id = frame_id
        raw_pub.publish(image_msg)

        # Publish compressed image if applicable
        #if compressed_pub is not None:
        #    compressed_image_msg = CompressedImage()
        #    compressed_image_msg.header = image_msg.header
        #    compressed_image_msg.format = 'jpeg'
        #    compressed_image_msg.data = cv2.imencode('.jpg', image)[1].tobytes()
        #    compressed_pub.publish(compressed_image_msg)


    def publish_camera_info(self, camera_info_pub, camera_info, camera_clock):
        # Update the timestamp
        camera_info.header.stamp = camera_clock
        camera_info_pub.publish(camera_info)


def main(args=None):
    rclpy.init(args=args)
    stereo_camera_node = StereoCameraNode()
    rclpy.spin(stereo_camera_node)
    stereo_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

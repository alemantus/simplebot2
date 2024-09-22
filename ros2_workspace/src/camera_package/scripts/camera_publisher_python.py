#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import time

class StereoCameraNode(Node):
 def __init__(self):
     super().__init__('stereo_camera_node')

     # Open video capture device using OpenCV
     self.cap = cv2.VideoCapture(0)  # Open /dev/video0
     if not self.cap.isOpened():
         self.get_logger().error("Failed to open /dev/video0")
         rclpy.shutdown()

     # Set the desired video frame width and height (1280x720)
     self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
     self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)

     # Create publishers for raw and compressed images
     self.pub_left = self.create_publisher(Image, 'stereo/left/image_raw', 10)
     self.pub_right = self.create_publisher(Image, 'stereo/right/image_raw', 10)
     self.pub_left_compressed = self.create_publisher(CompressedImage, 'stereo/left/image_raw/compressed', 10)
     self.pub_right_compressed = self.create_publisher(CompressedImage, 'stereo/right/image_raw/compressed', 10)

     # Timer to publish images at approximately 30 FPS
     self.timer = self.create_timer(0.1, self.publish_images)

     # Bridge to convert between ROS and OpenCV images
     self.bridge = CvBridge()

 def publish_images(self):
     ret, frame = self.cap.read()

     if ret:
         # Split the frame into left and right images
         left_image = frame[:, :frame.shape[1] // 2]
         right_image = frame[:, frame.shape[1] // 2:]

         # Publish left image
         self.publish_image(left_image, self.pub_left, self.pub_left_compressed, 'camera_left_frame')

         # Publish right image
         self.publish_image(right_image, self.pub_right, self.pub_right_compressed, 'camera_right_frame')

 def publish_image(self, image, raw_pub, compressed_pub, frame_id):
     # Convert OpenCV image to ROS Image message
     image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
     image_msg.header.stamp = self.get_clock().now().to_msg()
     image_msg.header.frame_id = frame_id
     raw_pub.publish(image_msg)

     # Publish compressed image
     compressed_image_msg = CompressedImage()
     compressed_image_msg.header = image_msg.header
     compressed_image_msg.format = 'jpeg'
     compressed_image_msg.data = cv2.imencode('.jpg', image)[1].tobytes()
     compressed_pub.publish(compressed_image_msg)

def main(args=None):
 rclpy.init(args=args)
 stereo_camera_node = StereoCameraNode()
 rclpy.spin(stereo_camera_node)
 stereo_camera_node.destroy_node()
 rclpy.shutdown()

if __name__ == '__main__':
 main()

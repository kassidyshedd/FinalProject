import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class FrameProcessor(Node):
    def __init__(self):
        super().__init__('frame_processor')
        self.get_logger().info('Initializing FrameProcessor node...')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.frame_captured = False
        self.get_logger().info('FrameProcessor node initialized.')

    def listener_callback(self, msg):
        self.get_logger().info('Received image message...')
        if not self.frame_captured:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Save the frame for further processing
            self.process_frame(cv_image)
            self.frame_captured = True
            rclpy.shutdown()  # Shutdown the node after capturing the frame

    def process_frame(self, frame):
        # Save the frame to disk as a PNG file
        save_path = 'captured_frame.png'
        cv2.imwrite(save_path, frame)
        self.get_logger().info(f'Frame saved at {os.path.abspath(save_path)}')

        # Open the saved PNG file for further processing
        self.open_saved_frame(save_path)

    def open_saved_frame(self, file_path):
        # Read the saved PNG file
        saved_frame = cv2.imread(file_path, cv2.IMREAD_COLOR)
        if saved_frame is not None:
            # Display the saved frame (optional)
            cv2.imshow('Saved Frame', saved_frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            # Further processing can be done here
            self.get_logger().info('Saved frame opened for further processing')
        else:
            self.get_logger().error('Failed to open saved frame')

def main(args=None):
    rclpy.init(args=args)
    processor = FrameProcessor()
    rclpy.spin(processor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

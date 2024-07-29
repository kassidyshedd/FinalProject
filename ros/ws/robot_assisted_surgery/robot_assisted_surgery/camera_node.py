import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np


class CameraNode(Node):

    def __init__(self):

        super().__init__('camera_node')
        self.bridge = CvBridge()

        # Initialize Realsense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)


        self.timer = self.create_timer(0.1, self.timer_callback)


    
    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return
        
        color_image = np.asanyarray(color_frame.get_data())
        ros_image = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        self.image_pub.publish(ros_image)

        cv2.imshow('Realsense', color_image)
        cv2.waitKey(1)

    
def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




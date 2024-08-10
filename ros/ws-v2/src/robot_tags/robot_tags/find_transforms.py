import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import apriltag
import numpy as np
import modern_robotics as mr
from geometry_msgs.msg import PointStamped

class FindTransforms(Node):

    def __init__(self):
        super().__init__('find_transforms')
        self.publisher = self.create_publisher(Image, 'uploaded_images', 10)
        self.point_publisher = self.create_publisher(PointStamped, 'transformed_point', 10)
        self.bridge = CvBridge()
        self.upload_and_detect_photos()

        self.tag_camera_object = None
        self.tag_camera_robot = None
        self.tag_topcamera_object = None
        self.tag_sidecamera_object = None

        self.P_world = None  

        self.clicked_point = None


    def create_translation_matrix(self, x, y, z):

        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]])


    def upload_and_detect_photos(self):
       
        photo_paths = [
            '/home/kashedd/finalproject/code/ros/ws-v2/captured_frame.png',
            '/home/kashedd/finalproject/pose_images/upload/fakeimg1.png',
            '/home/kashedd/finalproject/pose_images/upload/fakeimg2.png'
        ]
        
        for index, path in enumerate(photo_paths):
            if os.path.exists(path):
                self.get_logger().info(f'Uploading photo: {path}')
                image = cv2.imread(path)
                self.detect_apriltags(image, index)

                ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                self.publisher.publish(ros_image)
                self.get_logger().info(f'Photo {path} uploaded successfully.')

                if index == 1:
                    self.display_image_and_capture_click(image, path)
                else:
                    self.display_image(image, path)
            else:
                self.get_logger().error(f'Photo {path} does not exist.')


    def detect_apriltags(self, image, index):

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        options = apriltag.DetectorOptions(families="tag16h5")
        detector = apriltag.Detector(options)
        detections = detector.detect(gray)

        for detection in detections:
            self.draw_detection(image, detection)
            self.get_logger().info(f'Detected AprilTag: {detection.tag_id}')

            rvec, tvec = self.estimate_pose(detection)
            transform_matrix = self.create_transformation_matrix(rvec, tvec)

            detection_info = {
                "tag_id": detection.tag_id,
                "center": detection.center.tolist(),
                "corners": detection.corners.tolist(),
                "rvec": rvec.tolist(),
                "tvec": tvec.tolist(),
                "transform_matrix": transform_matrix.tolist()}

            # Livestream to object
            if index == 0 and detection.tag_id == 14:
                self.tag_camera_object = detection_info
                self.get_logger().info(f"T_camera_object: {transform_matrix}", once=True)

            # Livestream to robot
            elif index == 0 and detection.tag_id == 4:
                self.tag_camera_robot = detection_info
                self.get_logger().info(f"T_camera_robot: {transform_matrix}", once=True)

            elif index == 1:
                self.tag_topcamera_object = detection_info
                self.get_logger().info(f"T_top_object: {transform_matrix}", once=True)

            elif index == 2:
                self.tag_sidecamera_object = detection_info
                self.get_logger().info(f"T_camera_object: {transform_matrix}", once=True)

    def create_transformation_matrix(self, rvec, tvec):

        rotation_matrix, _ = cv2.Rodrigues(rvec)
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = tvec.flatten()

        return transform_matrix

    def estimate_pose(self, detection):

        tag_half_size = 0.1 / 2
        object_points = np.array([
            [-tag_half_size, -tag_half_size, 0],
            [ tag_half_size, -tag_half_size, 0],
            [ tag_half_size,  tag_half_size, 0],
            [-tag_half_size,  tag_half_size, 0]
        ])

        self.camera_matrix = np.array([[608.5162963867188, 0, 319.7140808105469],
                                       [0, 608.5162963867188, 248.23388671875],
                                       [0, 0, 1]])
        self.dist_coeffs = np.zeros(5)

        image_points = detection.corners
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)
        if not success:
            raise RuntimeError('Could not solve PnP')

        return rvec, tvec

    def transform_point(self, point, transform_matrix):

        point_h = np.array([point[0], point[1], point[2], 1.0])
        transformed_point_h = transform_matrix @ point_h
        transformed_point = transformed_point_h[:3] / transformed_point_h[3]

        return transformed_point

    def click_event(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_point = (x, y)
            self.get_logger().info(f"Clicked point: {self.clicked_point}")

            if self.tag_camera_object is None or self.tag_topcamera_object is None:
                self.get_logger().error("Transformation matrices are not available.")
                return

            # Assuming z = 0 for the clicked point in the image plane
            clicked_point_3d = np.array([x, y, 0.0])

            P_world = self.get_final(clicked_point_3d)
            self.get_logger().info(f"Transformed point in world frame: {P_world}")

            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = "world"
            point_msg.point.x = P_world[0] / 1000
            point_msg.point.y = P_world[1] / 1000
            point_msg.point.z = P_world[2] / 1000
            self.point_publisher.publish(point_msg)
        
            return P_world
        
    def invert_y(self):

        return np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])


    def get_final(self, p):

        P_top = p
        T_top_tag = np.array(self.tag_topcamera_object["transform_matrix"])
        T_live_tag = np.array(self.tag_camera_object["transform_matrix"])
        T_live_base = np.array(self.tag_camera_robot["transform_matrix"])
        T_world_base = self.create_translation_matrix(440, -30, 0)
        self.get_logger().info(f"T_world_base: {T_world_base}")

        T_world_live = np.dot(T_world_base, mr.TransInv(T_live_base))
        T_world_tag = np.dot(T_world_live, T_live_tag)
        T_world_top = np.dot(T_world_tag, mr.TransInv(T_top_tag))

        T_zfip = self.invert_y()
        T_zflip = np.dot(T_zfip, T_world_top)

        P_world = self.transform_point(P_top, T_zflip)
        
        return P_world


    def display_image_and_capture_click(self, image, window_name):
        cv2.imshow(window_name, image)
        cv2.setMouseCallback(window_name, self.click_event)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def draw_detection(self, image, detection):

        for i in range(4):
            cv2.line(image,
                     tuple(detection.corners[i-1, :].astype(int)),
                     tuple(detection.corners[i, :].astype(int)),
                     (0, 255, 0), 2)

        cv2.putText(image, str(detection.tag_id),
                    tuple(detection.center.astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 255), 2)
        

    def display_image(self, image, window_name):

        cv2.imshow(window_name, image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    photo_uploader = FindTransforms()
    rclpy.spin(photo_uploader)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from enum import Enum, auto
import yaml
import numpy as np
import cv2
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from robot.move_robot_functions import MoveRobotCalculations

from tf2_ros import TransformBroadcaster
from transformations import quaternion_from_matrix
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class State(Enum):
    TF = auto(),
    STOP = auto(),


class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot_node')

        config_rs_file = '/home/kashedd/finalproject/code/ros/ws_final/src/robot/config/realsense_intrinsics_2024_08_15.yaml'
        with open(config_rs_file, 'r') as file:
            config_rs = yaml.safe_load(file)
        config_top_file = '/home/kashedd/finalproject/code/ros/ws_final/src/robot/config/side_carm_intrinsics_2024_08_16.yaml'
        with open(config_top_file, 'r') as file:
            config_top= yaml.safe_load(file)
        config_side_file = '/home/kashedd/finalproject/code/ros/ws_final/src/robot/config/side_carm_intrinsics_2024_08_16.yaml'
        with open(config_side_file, 'r') as file:
            config_side = yaml.safe_load(file)

        self.rs_intrin = config_rs['camera_intrinsics']
        self.top_intrin = config_top['camera_intrinsics']
        self.side_intrin = config_side['camera_intrinsics']


        # Variables from YAML files
        self.rs_camera_mat = np.array([[self.rs_intrin['fx'], 0, self.rs_intrin['ppx']],
                                  [0, self.rs_intrin['fy'], self.rs_intrin['ppy']],
                                  [0, 0, 1]])
        self.rs_dist_coeffs = np.array([self.rs_intrin['coeffs']])
        self.top_camera_mat = np.array([[self.top_intrin['fx'], 0, self.top_intrin['ppx']],
                                  [0, self.top_intrin['fy'], self.top_intrin['ppy']],
                                  [0, 0, 1]])
        self.top_dist_coeffs = np.array([self.top_intrin['coeffs']])
        self.side_camera_mat = np.array([[self.side_intrin['fx'], 0, self.side_intrin['ppx']],
                                  [0, self.side_intrin['fy'], self.side_intrin['ppy']],
                                  [0, 0, 1]])
        self.side_dist_coeffs = np.array([self.side_intrin['coeffs']])

        self.tag_size = 0.075

        # Create instance of MoveRobotCalculations
        self.mrc = MoveRobotCalculations(self.rs_camera_mat, self.rs_dist_coeffs, 
                                         self.top_camera_mat, self.top_dist_coeffs,
                                         self.side_camera_mat, self.side_dist_coeffs, 
                                         self.tag_size)
        
        self.rs_tag4 = None
        self.rs_tag14 = None

        # Transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)

        # Publisher
        self.point_pub = self.create_publisher(PointStamped, 'entry', 10)
        self.traj_pub = self.create_publisher(Marker, 'trajectory', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Initialize Variables
        self.camera_to_base = None
        self.camera_to_object = None
        self.world_to_base = None   
        self.point = None

        self.state = State.TF

        # Photos 
        image1 = '/home/kashedd/Desktop/FinalProject/imageprocessig/DEMODAYTAG14.png'
        image2 = '/home/kashedd/Desktop/FinalProject/imageprocessig/DEMODAYTAGBOTH.png'
        top = '/home/kashedd/Desktop/FinalProject/at-compute/savedxray2/IM00002.png'
        side = '/home/kashedd/Desktop/FinalProject/at-compute/savedxray2/IM00001.png'

        self.photo_array = [image1, image2, top, side]
        self.mrc.open_images(self.photo_array)


    
    def timer_callback(self):
        
        if self.state == State.TF:
            camera_frame = "camera_color_optical_frame"
            tag4_frame = "tag4"
            tag14_frame = "tag14"
            world_frame = "world"

            # try:
            #     t = self.tf_buffer.lookup_transform(camera_frame, tag4_frame, rclpy.time.Time())
            #     self.camera_to_base = [
            #         t.transform.translation.x,
            #         t.transform.translation.y,
            #         t.transform.translation.z,
            #         t.transform.rotation.x,
            #         t.transform.rotation.y,
            #         t.transform.rotation.z,
            #         t.transform.rotation.w]
                
            #     # self.get_logger().info(f"Transform c2b: {self.camera_to_base}")
                
            # except TransformException as ex:
            #     self.get_logger().info(f"Could not get transform {tag4_frame} to {camera_frame}: {ex}")

            # try:
            #     t = self.tf_buffer.lookup_transform(camera_frame, tag14_frame, rclpy.time.Time())
            #     self.camera_to_object = [
            #         t.transform.translation.x,
            #         t.transform.translation.y,
            #         t.transform.translation.z,
            #         t.transform.rotation.x,
            #         t.transform.rotation.y,
            #         t.transform.rotation.z,
            #         t.transform.rotation.w]
                
            #     # self.get_logger().info(f"Transform c2o: {self.camera_to_object}")
                
            # except TransformException as ex:
            #     self.get_logger().info(f"Could not get transform {tag14_frame} to {camera_frame}: {ex}")

            # try:
            #     t = self.tf_buffer.lookup_transform(world_frame, tag4_frame, rclpy.time.Time())
            #     self.world_to_base = [
            #         t.transform.translation.x,
            #         t.transform.translation.y,
            #         t.transform.translation.z,
            #         t.transform.rotation.x,
            #         t.transform.rotation.y,
            #         t.transform.rotation.z,
            #         t.transform.rotation.w]
            #     # self.get_logger().info(f"Transform w2b: {self.world_to_base}")
                
            # except TransformException as ex:
            #     self.get_logger().info(f"Could not get transform {tag4_frame} to {world_frame}: {ex}")

            point = self.mrc.get_point()
            self.point = point
            # self.get_logger().info(f"Point: {self.point}")
            if point is not None:
                pt = PointStamped()
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.header.frame_id = 'world'
                pt.point.x = point[0][0]
                pt.point.y = point[1][0]
                pt.point.z = point[2][0]
                self.point_pub.publish(pt)

                m = Marker()
                m.header.frame_id = 'world'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'traj'
                m.id = 0
                m.type = Marker.ARROW
                m.action = Marker.ADD
                m.scale.x = 0.01
                m.scale.y = 0.02
                m.color.r = 1.0
                m.color.a = 1.0

                # Entry
                ep = Point()
                ep.x = point[0][0]
                ep.y = point[1][0]
                ep.z = point[2][0]
                
                # Target
                tp = Point()
                tp.x = point[0][0] + 0.13
                tp.y = point[1][0] -0.13
                tp.z = point[2][0] - 0.1

                m.points.append(ep)
                m.points.append(tp)

                self.traj_pub.publish(m)





            # tw = TransformStamped()

            # # Set header
            # tw.header.stamp = self.get_clock().now().to_msg()
            # tw.header.frame_id = 'tag4_frame'  # Reference frame
            # tw.child_frame_id = 'world'  # Tag frame

            # # Set translation (tvec)
            # tw.transform.translation.x = 0.0
            # tw.transform.translation.y = -0.1141
            # tw.transform.translation.z = 0.0

            # # Set rotation (quaternion)
            # tw.transform.rotation.x = 0.0
            # tw.transform.rotation.y = 0.0
            # tw.transform.rotation.z = 0.0
            # tw.transform.rotation.w = 1.0

            # # Publish the transform
            # self.br.sendTransform(tw)

            
            # rs_tag4 = self.mrc.get_rs_tag4() 
            # self.tvec = rs_tag4['tvec']  
            # rotation_matrix, _ = cv2.Rodrigues(np.array(rs_tag4['rvec_corrected']))
            # T = np.eye(4)
            # T[:3, :3] = rotation_matrix

            # # Convert the rotation matrix to a quaternion
            # quaternion = quaternion_from_matrix(T)

            # # Create a TransformStamped message
            # t = TransformStamped()

            # # Set header
            # t.header.stamp = self.get_clock().now().to_msg()
            # t.header.frame_id = 'camera_frame'  # Reference frame
            # t.child_frame_id = 'tag4_frame'  # Tag frame

            # # Set translation (tvec)
            # t.transform.translation.x = self.tvec[0][0]
            # t.transform.translation.y = self.tvec[1][0]
            # t.transform.translation.z = self.tvec[2][0]

            # # Set rotation (quaternion)
            # t.transform.rotation.x = quaternion[0]
            # t.transform.rotation.y = quaternion[1]
            # t.transform.rotation.z = quaternion[2]
            # t.transform.rotation.w = quaternion[3]

            # # Publish the transform
            # self.br.sendTransform(t)

            # rs_tag14 = self.mrc.get_rs_tag14() 
            # self.tvec1 = rs_tag14['tvec']  
            # rotation_matrix1, _ = cv2.Rodrigues(np.array(rs_tag14['rvec']))
            # T1 = np.eye(4)
            # T1[:3, :3] = rotation_matrix1
            # quaternion1 = quaternion_from_matrix(T1)


            # tz = TransformStamped()

            # # Set header
            # tz.header.stamp = self.get_clock().now().to_msg()
            # tz.header.frame_id = 'camera_frame'  # Reference frame
            # tz.child_frame_id = 'tag14_frame'  # Tag frame

            # # Set translation (tvec)
            # tz.transform.translation.x = self.tvec1[0][0]
            # tz.transform.translation.y = self.tvec1[1][0]
            # tz.transform.translation.z = self.tvec1[2][0]

            # # Set rotation (quaternion)
            # tz.transform.rotation.x = quaternion1[0]
            # tz.transform.rotation.y = quaternion1[1]
            # tz.transform.rotation.z = quaternion1[2]
            # tz.transform.rotation.w = quaternion1[3]

            # # Publish the transform
            # self.br.sendTransform(tz)
            # # self.get_logger.info("Publishing?")


            # # Send transforms to MRC
            # self.mrc.send_tfs(self.camera_to_base, self.camera_to_object, self.world_to_base)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
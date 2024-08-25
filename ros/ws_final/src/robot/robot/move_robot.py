import rclpy
from rclpy.node import Node
from enum import Enum, auto
import yaml
import numpy as np
from robot.move_robot_functions import MoveRobotCalculations

from tf2_ros import TransformBroadcaster
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
                tp.x = point[0][0] 
                tp.y = point[1][0] 
                tp.z = point[2][0] 

                m.points.append(ep)
                m.points.append(tp)

                self.traj_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
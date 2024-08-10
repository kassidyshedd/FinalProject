import rclpy 
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from enum import Enum, auto
from scipy.spatial.transform import Rotation as R
import numpy as np
import modern_robotics as mr
import tf_transformations
import time


class State(Enum):

    GET_TF = auto(),
    CALC = auto(),
    INIT = auto(),


class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot')
        
        # Transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.camera_to_base = None
        self.camera_to_object = None
        self.world_to_base = None

        self.state = State.INIT


    def quat_to_mat(self, trans, rot):

        T = tf_transformations.quaternion_matrix(rot)
        T[:3, 3] = trans
        
        return T
    
    def mat_to_quat(self, T):

        trans = T[:3, 3]
        rot = tf_transformations.quaternion_from_matrix(T)

        return trans, rot    

    def timer_callback(self):

        camera_frame = "camera_color_optical_frame"
        base_frame = "base_tag"
        object_frame = "object_tag"
        world_frame = "world"

        if self.state == State.INIT:
            time.sleep(5)
            self.state = State.GET_TF

        if self.state == State.GET_TF:
            try:
                t = self.tf_buffer.lookup_transform(camera_frame, base_frame, rclpy.time.Time())
                self.camera_to_base = [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w]
                
                self.get_logger().info(f"Transform c2b: {self.camera_to_base}")
                
            except TransformException as ex:
                self.get_logger().info(f"Could not get transform {base_frame} to {camera_frame}: {ex}")

            try:
                t = self.tf_buffer.lookup_transform(camera_frame, object_frame, rclpy.time.Time())
                self.camera_to_object = [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w]
                
                self.get_logger().info(f"Transform c2o: {self.camera_to_object}")
                
            except TransformException as ex:
                self.get_logger().info(f"Could not get transform {object_frame} to {camera_frame}: {ex}")

            
            try:
                t = self.tf_buffer.lookup_transform(world_frame, base_frame, rclpy.time.Time())
                self.world_to_base = [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w]
                s4
                self.get_logger().info(f"Transform w2b: {self.world_to_base}")
                
            except TransformException as ex:
                self.get_logger().info(f"Could not get transform {base_frame} to {world_frame}: {ex}")

            
            if self.camera_to_base and self.camera_to_object is not None:
                self.state = State.CALC

        if self.state == State.CALC:

            t_w2b = np.array([self.world_to_base[0], self.world_to_base[1], self.world_to_base[2]])
            r_w2b = np.array([self.world_to_base[3], self.world_to_base[4], self.world_to_base[5], self.world_to_base[6]])
            
            t_c2b = np.array([self.camera_to_base[0], self.camera_to_base[1], self.camera_to_base[2]])
            r_c2b = np.array([self.camera_to_base[3], self.camera_to_base[4], self.camera_to_base[5], self.camera_to_base[6]])


            t_c2o = np.array([self.camera_to_object[0], self.camera_to_object[1], self.camera_to_object[2]])
            r_c2o = np.array([self.camera_to_object[3], self.camera_to_object[4], self.camera_to_object[5], self.camera_to_object[6]])

            # Get transformation matrix
            T_world_base = self.quat_to_mat(t_w2b, r_w2b)
            T_camera_base = self.quat_to_mat(t_c2b, r_c2b)
            T_camera_object = self.quat_to_mat(t_c2o, r_c2o)

            # T_w2o = T_wb * T_cb.inv * T_co
            # Get T_cb.inv
            T_base_camera = mr.TransInv(T_camera_base)
            # T_base_object = np.dot(T_base_camera, T_camera_object)

            # Get T_wb * T_cb.inv
            T_world_camera = np.dot(T_world_base, T_base_camera)

            # Get T_w2o
            T_world_object = np.dot(T_world_camera, T_camera_object)
            

            t, r = self.mat_to_quat(T_world_object)

            # t, r = self.mat_to_quat(T_base_object)
            ang = tf_transformations.euler_from_quaternion(r)

            self.get_logger().info(f"T_base_object -- trans: {t}", once=True)
            self.get_logger().info(f"T_base_object -- angles: {ang}", once=True)



def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

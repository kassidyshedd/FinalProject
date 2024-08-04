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
import transformations as tr


class State(Enum):

    GET_TF = auto(),
    CALC = auto(),


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

        self.state = State.GET_TF


    def t_mat(self, tf):

        tx, ty, tz = tf[:3]
        qx, qy, qz, qw = tf[3:]

        rotation = R.from_quat([qx, qy, qz, qw]).as_matrix()

        tm = np.eye(4)
        tm[:3, :3] = rotation
        tm[:3, 3] = [tx, ty, tz]

        return tm
    
    def t(self, p, q):
        norm = np.linalg.norm(q)
        q = q / norm
        g = tr.quaternion_matrix(q)
        g[0:3, -1] = p

        return g



    def timer_callback(self):

        camera_frame = "camera_color_optical_frame"
        base_frame = "base_tag"
        object_frame = "object_tag"

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

            # if self.camera_to_base and self.camera_to_object is not None:
            #     self.state = State.CALC

        if self.state == State.CALC:
            
            p_c2b = np.array([self.camera_to_base[0], self.camera_to_base[1], self.camera_to_base[2]])
            q_c2b = np.array([self.camera_to_base[3], self.camera_to_base[4], self.camera_to_base[5], self.camera_to_base[6]])
            T_c2b = self.t(p_c2b, q_c2b)

            p_c2o = np.array([self.camera_to_object[0], self.camera_to_object[1], self.camera_to_object[2]])
            q_c2o = np.array([self.camera_to_object[3], self.camera_to_object[4], self.camera_to_object[5], self.camera_to_object[6]])
            T_c2o = self.t(p_c2o, q_c2o)

            T_b2c = mr.TransInv(T_c2b)
            T_b2o = np.dot(T_b2c, T_c2o)

            self.get_logger().info(f"tm: {T_b2o}")


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

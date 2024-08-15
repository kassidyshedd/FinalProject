import rclpy 
from rclpy.node import Node
import datetime
import os
from enum import Enum, auto

# for realsense
import pyrealsense2 as rs

class State(auto):
    START = auto(),
    STOP = auto(),


"""
Performs 3 types of camera calibration:
    1. realsense
    2. top_carm
    3. side_carm
"""

class CameraCalibration(Node):

    def __init__(self):
        super().__init__('camera_calibration_node')

        # Timer
        self.timer = self.create_timer(1, self.timer_callback)

        # Declare a parameter, default is set to the intel realsense
        self.declare_parameter('camera_type', 'realsense')

        self.state = State.START


    def timer_callback(self):

        if self.state == State.START:
            # Get parameter
            param = self.get_parameter('camera_type').get_parameter_value().string_value
            self.get_logger().info(f"Calibrating Intrinsics for Camera Type: {param}")

            if param == 'realsense':
                # 1. Make sure intel realsense is running
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

                try:
                    pipeline.start(config)
                    frames = pipeline.wait_for_frames()
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        self.get_logger().error("No frame found.")

                    # 2. Get intrinsics
                    intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

                    # 3. Save as yaml - realsense_intrinsics{date}.yaml
                    data = {
                        'wdith': intrinsics.width,
                        'height': intrinsics.height,
                        'ppx': intrinsics.ppx,
                        'ppy': intrinsics.ppy,
                        'fx': intrinsics.fx,
                        'fy': intrinsics.fy,
                        'model': intrinsics.model,
                        'coeffs': intrinsics.coeffs}
                    
                    config_dir = os.path.join(os.path.expanduser('~'), 'finalproject/code/ros/ws_final/robot/config')
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)
            
                    cur_time = datetime.datetime.now().strftime("%Y_%m_%d")
                    file_name = f"realsense_intrinsics_{cur_time}.config.yaml"
                    file_path = os.path.join(config_dir, file_name)

                    with open(file_path, 'w') as file:
                        for key, value in data.items():
                            file.write(f"{key}: {value}\n")
                    self.get_logger().info(f"Intrinsics saved to: {file_path}")

                finally:
                    pipeline.stop()

            if param == 'top_carm':
                # 1. Load all images from folder /top-carm
                # 2. Convert dicom to png
                # 3. Blob Detection
                # 4. Calculate Intrinsics
                # 5. Save as yaml - top_carm_intrinsics{date}.yaml
                pass

            if param == 'side_carm':
                # 1. Load all images from folder /side-carm
                # 2. Convert dicom to png
                # 3. Blob Detection
                # 4. Calculate Intrinsics
                # 5. Save as yaml - side_carm_intrinsics{date}.yaml
                pass

            self.state = State.STOP

        if self.state == State.STOP:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibration()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
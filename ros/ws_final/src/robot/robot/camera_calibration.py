import rclpy 
from rclpy.node import Node
import datetime
import os
from enum import Enum, auto
import numpy as np
import cv2
import json
from robot.blob_detection_functions import BlobDetection 


# for realsense
import pyrealsense2 as rs

# for C-Arm
import pydicom
from PIL import Image


class State(Enum):
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

    def dicom_to_png(self, dicom, png):
        dicom_img = pydicom.dcmread(dicom)
        pixel_array = dicom_img.pixel_array

        norm_array = (pixel_array - np.min(pixel_array)) / (np.max(pixel_array) - np.min(pixel_array)) * 255
        norm_array = norm_array.astype(np.uint8)

        png_img = Image.fromarray(norm_array)
        png_img.save(png)

    def compute_reprojection_error(self, obj_points, img_points, rvecs, tvecs, camera_matrix, dist_coeffs):
        total_error = 0
        for i in range(len(obj_points)):
            img_points2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
            
            # Ensure img_points[i] and img_points2 have the same shape
            img_points_actual = np.array(img_points[i], dtype=np.float32).reshape(-1, 2)
            img_points_projected = img_points2.reshape(-1, 2)

            # Calculate the reprojection error
            error = cv2.norm(img_points_actual, img_points_projected, cv2.NORM_L2) / len(img_points_projected)
            total_error += error
        
        mean_error = total_error / len(obj_points)
        return mean_error

    def timer_callback(self):

        if self.state == State.START:
            # Get parameter
            param = self.get_parameter('camera_type').get_parameter_value().string_value
            self.get_logger().info(f"Calibrating Intrinsics for Camera Type: {param}")

            if param == 'realsense':
                # 1. Make sure intel realsense is running
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
                config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

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
                        'ppx': intrinsics.ppx,
                        'ppy': intrinsics.ppy,
                        'fx': intrinsics.fx,
                        'fy': intrinsics.fy,
                        'coeffs': intrinsics.coeffs}
                    
                    config_dir = os.path.join(os.path.expanduser('~'), 'finalproject/code/ros/ws_final/src/robot/config')
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

            if param == 'top_carm' or 'side_carm':
                # 1. Load all images from folder /top-carm or /side-car,
                if param == 'top_carm':
                    image_dir = os.path.join(os.path.expanduser('~'), 'finalproject/code/ros/ws_final/src/robot/images/dicom/top_carm')
                    save_dir = os.path.join(os.path.expanduser('~'), 'finalproject/code/ros/ws_final/src/robot/images/png/top_carm')
                    cur_time = datetime.datetime.now().strftime("%Y_%m_%d")
                    file_name = f"top_carm_intrinsics_{cur_time}.yaml"
                    if not os.path.exists(save_dir):
                        os.makedirs(save_dir)

                if param == 'side_carm':
                    image_dir = os.path.join(os.path.expanduser('~'), 'finalproject/code/ros/ws_final/src/robot/images/dicom/side_carm')
                    save_dir = os.path.join(os.path.expanduser('~'), 'finalproject/code/ros/ws_final/src/robot/images/png/side_carm')
                    cur_time = datetime.datetime.now().strftime("%Y_%m_%d")
                    file_name = f"side_carm_intrinsics_{cur_time}.yaml"
                    if not os.path.exists(save_dir):
                        os.makedirs(save_dir)

                # 2. Convert dicom to png
                for filename in os.listdir(image_dir):
                    if filename.endswith(".dcm"):
                        dicom_path = os.path.join(image_dir, filename)
                        png_filename = os.path.splitext(filename)[0] + ".png"
                        png_path = os.path.join(save_dir, png_filename)
                        self.dicom_to_png(dicom_path, png_path)
                        
                # 3. Blob Detection / saves marker centers
                blob_detector = BlobDetection(save_dir)
                width, height, channels = blob_detector.get_image_size(png_path)
                blob_detector.run()

                # SAVING FIX_LATER
                with open('marker_centers.json', 'r') as file:
                    data = json.load(file)
                fiducial1 = []
                fidicual2 = []
                fiducial3 = []
                for entry in data:
                    image_path = entry[0]
                    points = entry[1]
                    fiducial1.append([image_path, points[:14]])
                    fidicual2.append([image_path, points[14:25]])
                    fiducial3.append([image_path, points[25:]])
                with open('fiducial1.json', 'w') as f:
                    json.dump(fiducial1, f)
                with open('fiducial2.json', 'w') as f:
                    json.dump(fidicual2, f)
                with open('fiducial3.json', 'w') as f:
                    json.dump(fiducial3, f)

                # 4. Calculate Intrinsics
                all_fid1_pts = []
                fid1_obj_pts = []
                all_fid2_pts = []
                fid2_obj_pts = []
                all_fid3_pts = []
                fid3_obj_pts = []
                model1 = np.array([
                    [2.5, 2.5, 21], [7.5, 42.5, 21], [42.5, 37.5, 21], # Layer #1
                    [7.5, 22.5, 16], [17.5, 37.5, 16], [37.5, 32.5, 16], [32.5, 7.5, 16], # Layer #2
                    [12.5, 17.5, 11], [27.5, 32.5, 11], [32.5, 22.5, 11], [17.5, 12.5, 11], # Layer #3
                    [17.5, 17.5, 6], [27.5, 27.5, 6], [22.5, 22.5, 6]], dtype=np.float32)

                model2 = np.array([
                    [2.5, 7.5, 21], [7.5, 42.5, 21], [42.5, 22.5, 21], [22.5, 2.5, 21], # Layer #1
                    [7.5, 7.5, 16], [37.5, 37.5, 16], # Layer #2
                    [12.5, 32.5, 11], [32.5, 12.5, 11], # Layer #3
                    [17.5, 22.5, 6], [22.5, 27.5, 6], [27.5, 17.5, 6]], dtype=np.float32)

                model3 = np.array([
                    [2.5, 42.5, 21], [42.5, 2.5, 21], # Layer #1
                    [7.5, 37.5, 16], [37.5, 17.5, 16], [22.5, 7.5, 16], # Layer #2
                    [12.5, 17.5, 11], [17.5, 32.5, 11], [32.5, 27.5, 11], [27.5, 12.5, 11], # Layer #3
                    [17.5, 27.5, 6], [27.5, 22.5, 6], [22.5, 17.5, 6]], dtype=np.float32)


                fid1_centers = blob_detector.open_json('fiducial1.json')
                fid2_centers = blob_detector.open_json('fiducial2.json')
                fid3_centers = blob_detector.open_json('fiducial3.json')

                for path, mc in fid1_centers:
                    fid1_obj_pts.append(model1)
                    all_fid1_pts.append(np.array(mc, dtype=np.float32))
                    
                for path, mc in fid2_centers:
                    fid2_obj_pts.append(model2)
                    all_fid2_pts.append(np.array(mc, dtype=np.float32))

                for path, mc in fid3_centers:
                    fid3_obj_pts.append(model3)
                    all_fid3_pts.append(np.array(mc, dtype=np.float32))

                img_size = (width, height)

                # Initial Intrinsics Guess
                focal_len = img_size[0]
                c = (img_size[0] / 2, img_size[1] / 2)
                guess_cm = np.array([
                    [focal_len, 0, c[0]],
                    [0, focal_len, c[1]],
                    [0, 0, 1]])
                guess_dc = np.zeros(5)

                camera_matrices = []
                distortion_coeffs = []
                reprojection_errors = []

                ret1, camera_mat1, dist_coeffs1, rvecs1, tvecs1 = cv2.calibrateCamera(fid1_obj_pts, all_fid1_pts, img_size, guess_cm, guess_dc, None, None, cv2.CALIB_USE_INTRINSIC_GUESS)
                error1 = self.compute_reprojection_error(fid1_obj_pts, all_fid1_pts, rvecs1, tvecs1, camera_mat1, dist_coeffs1)
                camera_matrices.append(camera_mat1)
                distortion_coeffs.append(dist_coeffs1)
                reprojection_errors.append(error1)
                self.get_logger().info(f"FIDUCIAL #1\nCamera Matrix:\n{camera_mat1}")
                self.get_logger().info(f"Distortion Coeff:\n{dist_coeffs1}")
                self.get_logger().info(f"Reprojection Error: {error1}")

                ret2, camera_mat2, dist_coeffs2, rvecs2, tvecs2 = cv2.calibrateCamera(fid2_obj_pts, all_fid2_pts, img_size, guess_cm, guess_dc, None, None, cv2.CALIB_USE_INTRINSIC_GUESS)
                error2 = self.compute_reprojection_error(fid2_obj_pts, all_fid2_pts, rvecs2, tvecs2, camera_mat2, dist_coeffs2)
                camera_matrices.append(camera_mat2)
                distortion_coeffs.append(dist_coeffs2)
                reprojection_errors.append(error2)
                self.get_logger().info(f"FIDUCIAL #2\nCamera Matrix:\n{camera_mat2}")
                self.get_logger().info(f"Distortion Coeff:\n{dist_coeffs2}")
                self.get_logger().info(f"Reprojection Error: {error2}")

                ret3, camera_mat3, dist_coeffs3, rvecs3, tvecs3 = cv2.calibrateCamera(fid3_obj_pts, all_fid3_pts, img_size, guess_cm, guess_dc, None, None, cv2.CALIB_USE_INTRINSIC_GUESS)
                error3 = self.compute_reprojection_error(fid3_obj_pts, all_fid3_pts, rvecs3, tvecs3, camera_mat3, dist_coeffs3)
                camera_matrices.append(camera_mat3)
                distortion_coeffs.append(dist_coeffs3)
                reprojection_errors.append(error3)
                self.get_logger().info(f"FIDUCIAL #3\nCamera Matrix:\n{camera_mat3}")
                self.get_logger().info(f"Distortion Coeff:\n{dist_coeffs3}")
                self.get_logger().info(f"Reprojection Error: {error3}")

                min_error = min(reprojection_errors)
                max_error = max(reprojection_errors)

                if max_error - min_error > 2:
                    self.get_logger().info(f"Reprojection error difference > 2, dropping the highest error.")
                    # Find the index of the highest error and remove it
                    max_index = reprojection_errors.index(max_error)
                    camera_matrices.pop(max_index)
                    distortion_coeffs.pop(max_index)
                else:
                    self.get_logger().info(f"Reprojection error difference <= 2, averaging all.")

                # Average the remaining camera matrices and distortion coefficients
                avg_camera_matrix = np.mean(camera_matrices, axis=0)
                avg_dist_coeffs = np.mean(distortion_coeffs, axis=0)

                self.get_logger().info(f"Averaged Camera Matrix:\n{avg_camera_matrix}")
                self.get_logger().info(f"Averaged Distortion Coefficients:\n{avg_dist_coeffs}")
                      
                # 5. Save as yaml - top_carm_intrinsics{date}.yaml
                data = {
                    'ppx': avg_camera_matrix[0, 2],
                    'ppy': avg_camera_matrix[1, 2],
                    'fx': avg_camera_matrix[0, 0],
                    'fy': avg_camera_matrix[1, 1],
                    'coeffs': avg_dist_coeffs.flatten().tolist()}

                config_dir = os.path.join(os.path.expanduser('~'), 'finalproject/code/ros/ws_final/src/robot/config')
                if not os.path.exists(config_dir):
                    os.makedirs(config_dir)
                file_path = os.path.join(config_dir, file_name)

                with open(file_path, 'w') as file:
                    for key, value in data.items():
                        file.write(f"{key}: {value}\n")
                self.get_logger().info(f"Intrinsics saved to: {file_path}")

            self.state = State.STOP

        if self.state == State.STOP:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibration()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
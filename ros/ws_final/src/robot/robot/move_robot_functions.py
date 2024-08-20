import os
import cv2
import apriltag
import numpy as np
import modern_robotics as mr

class MoveRobotCalculations:
    def __init__(self, rs_camera_mat, rs_dist_coeffs, top_camera_mat, top_dist_coeffs, side_camera_mat, side_dist_coeffs, tag_size):

        # Intrinsics of cameras
        self.rs_camera_mat = rs_camera_mat
        self.rs_dist_coeffs = rs_dist_coeffs
        self.top_camera_mat = top_camera_mat
        self.top_dist_coeffs = top_dist_coeffs
        self.side_camera_mat = side_camera_mat
        self.side_dist_coeffs = side_dist_coeffs
        self.tag_size = tag_size

        # Apriltag / fiducial values
        self.rs_tag14_forclick = None
        self.rs_tag14 = None
        self.rs_tag4 = None

        self.img1_pt = None
        self.img2_pt = None

        self.camera_to_tag4 = None
        self.camera_to_tag14 = None
        self.world_to_tag4 = None

    
    def get_rs_tag4(self):
        return self.rs_tag4
    
    def get_rs_tag14(self):
        return self.rs_tag14
    
    def send_tfs(self, c24, c214, w24):
        pass


    def open_images(self, photo_array):
        for index, path in enumerate(photo_array):
            if os.path.exists(path):
                image = cv2.imread(path)
                self.detect_tags(image, index)
                self.display_image_click_capture(image, path, index)

    
    def detect_tags(self, image, index):
        # First 2 photos will use apriltag library (taken with realsense)
        if index == 0 or index == 1:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            options = apriltag.DetectorOptions(families="tag16h5")
            detector = apriltag.Detector(options)
            detections = detector.detect(gray)

            for detection in detections:
                self.draw_detection(image, detection)
                rvec, tvec = self.estimate_pose_rs(detection)
                T_mat = self.create_transformation_matrix(rvec, tvec)

                # Need to get corrected rvec and tvec
                R, _ = cv2.Rodrigues(rvec)
                R_c = np.dot(R, self.rotate_x_180())
                rvec_c, _ = cv2.Rodrigues(R_c)
                r_T_mat = self.create_transformation_matrix(rvec_c, tvec)

                detection_info = {
                    "tag_id": detection.tag_id,
                    "center": detection.center.tolist(),
                    "corners": detection.corners.tolist(),
                    "rvec": rvec.tolist(),
                    "tvec": tvec.tolist(),
                    "rvec_corrected": rvec_c.tolist(),
                    "transformation_matrix": T_mat,
                    "rotated_transformation_matrix": r_T_mat}

                # Livestream photo with only tag 14 
                if index == 0 and detection.tag_id == 14:
                    self.rs_tag14_forclick = detection_info
                    print(self.rs_tag14_forclick['center'])
                # Livestream photo with tag4 and tag14
                if index == 1 and detection.tag_id == 4:
                    self.rs_tag4 = detection_info
                    # print(self.rs_tag4['center'])
                if index == 1 and detection.tag_id == 14:
                    self.rs_tag14 = detection_info
                    print(self.rs_tag14['center'])

                self.draw_axes(image, detection.center, T_mat, r_T_mat)


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
            
    
    def rotate_x_180(self):
        return np.array([
            [1, 0, 0],
            [0, -1, 0], 
            [0, 0, -1]])
    

    def mat_from_tf(self, tf):
        pass
             

    def draw_axes(self, image, center, transform_matrix, r_transform_matrix):
        axis_len = 0.1
        axis_pts = np.float32([
            [axis_len, 0, 0],
            [0, axis_len, 0],
            [0, 0, axis_len],
            [0, 0, 0]]).reshape(-1, 3)
        
        center = tuple(map(int, center))
        
        # Original Coordiate Frame
        # img_pts, _ = cv2.projectPoints(axis_pts, transform_matrix[:3, :3], transform_matrix[:3, 3], self.rs_camera_mat, self.rs_dist_coeffs)
        # img_pts = img_pts.astype(int)
        # cv2.line(image, center, tuple(img_pts[0].ravel()), (0, 0, 255), 3)  # X-axis in red
        # cv2.line(image, center, tuple(img_pts[1].ravel()), (0, 255, 0), 3)  # Y-axis in green
        # cv2.line(image, center, tuple(img_pts[2].ravel()), (255, 0, 0), 3)  # Z-axis in blue

        # Corrected Coordinate Frame (rotate 180 degress around x-axis)
        r_img_pts, _ = cv2.projectPoints(axis_pts, r_transform_matrix[:3, :3], r_transform_matrix[:3, 3], self.rs_camera_mat, self.rs_dist_coeffs)
        r_img_pts = r_img_pts.astype(int)
        cv2.line(image, center, tuple(r_img_pts[0].ravel()), (0, 0, 200), 1, lineType=cv2.LINE_AA)  # X-axis in red (rotated)
        cv2.line(image, center, tuple(r_img_pts[1].ravel()), (0, 200, 0), 1, lineType=cv2.LINE_AA)  # Y-axis in green (rotated)
        cv2.line(image, center, tuple(r_img_pts[2].ravel()), (200, 0, 0), 1, lineType=cv2.LINE_AA)  # Z-axis in blue (rotated)

    
    def display_image_click_capture(self, image, window_name, index):
        cv2.imshow(window_name, image)
        cv2.setMouseCallback(window_name, lambda event, x, y, flags, params: self.click_event(event, x, y, flags, params, index))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    
    def estimate_pose_rs(self, detection):
        ts = self.tag_size / 2
        obj_pts = np.array([
            [-ts, -ts, 0],
            [ts, -ts, 0],
            [ts, ts, 0],
            [-ts, ts, 0]])
        img_pts = detection.corners
        _, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, self.rs_camera_mat, self.rs_dist_coeffs)
        return rvec, tvec
    
    
    def create_transformation_matrix(self, rvec, tvec):
        rotation_mat, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = rotation_mat
        T[:3, 3] = tvec.flatten()
        return T
    

    def click_event(self, event, x, y, flags, params, index):
        if event == cv2.EVENT_LBUTTONDOWN:
            if index == 0:
                self.img1_pt = (x, y)
                print(f"Clicked Point in Image 1: {self.img1_pt}")
            if index == 1:
                self.img2_pt = (x, y)
                print(f"Clicked Point in Image 2: {self.img2_pt}")

            if self.img1_pt and self.img2_pt is not None:
                P_world = self.triangulate_pts()

    
    def triangulate_pts(self):

        # # Make click homogenous
        # img1_pt_h = np.array([[self.img1_pt[0]], [self.img1_pt[1]]])
        # img2_pt_h = np.array([[self.img2_pt[0]], [self.img2_pt[1]]])

        # # Create projection matrix
        # P1 = np.dot(self.rs_camera_mat, self.rs_tag14_forclick['rotated_transformation_matrix'][:3, :])
        # P2 = np.dot(self.rs_camera_mat, self.rs_tag14['rotated_transformation_matrix'][:3, :])

        # # Triangulate Point
        # point_4d_h = cv2.triangulatePoints(P1, P2, img1_pt_h, img2_pt_h)
        # print(f"point4d: {point_4d_h}")
        # point_3d = point_4d_h[:3] / point_4d_h[3]
        # point3d_h = np.append(point_3d, 1)
        # print(f"Triangulated 3D Point in Camera Frame: {point_3d}")

        # # Move to image2cam frame 
        # T_image1cam_tag14 = self.rs_tag14_forclick['rotated_transformation_matrix']
        # T_image2cam_tag14 = self.rs_tag14['rotated_transformation_matrix']
        # T_tag14_img1cam = mr.TransInv(T_image1cam_tag14)
        # T_img2cam_img1cam = np.dot(T_image2cam_tag14, T_tag14_img1cam)
        # P_img2cam_h = np.dot(T_img2cam_img1cam, point3d_h)
        # print(f"Point in image2cam frame: {P_img2cam_h}")

        # # Move to tag4 frame
        # T_image2cam_tag4 = self.rs_tag4['rotated_transformation_matrix']
        # T_tag4_img2cam = mr.TransInv(T_image2cam_tag4)
        # P_tag4_h = np.dot(T_tag4_img2cam, P_img2cam_h)
        # print(f"Point in tag4 frame: {P_tag4_h}")

        # # Move to world frame 
        T_world_tag4 = np.array([
            [1, 0, 0, 0], 
            [0, 1, 0, -0.1141],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        
        # T_tag4_world = mr.TransInv(T_world_tag4)
        # P_world_h = np.dot(T_world_tag4, P_tag4_h)
        # print(f"Point in world frame: {P_world_h}")

        # Testing Transforms
        # 1. I want P_tag4 -- P_world = [0.1, 0.1, 0.1]
        P_world = np.array([0.125, -0.2, 0.085, 1])
        print(f"P_world: {P_world}")
        T_tag4_world = mr.TransInv(T_world_tag4)
        P_tag4 = np.dot(T_tag4_world, P_world)
        print(f"P_tag4: {P_tag4}")

        # 2. I want P_camera
        T_img2cam_tag4 = self.rs_tag4['rotated_transformation_matrix']
        # print(T_camera_tag4)
        P_img2cam = np.dot(T_img2cam_tag4, P_tag4)
        print(f"P_camera: {P_img2cam}")
        img2_pt_h = np.dot(self.rs_camera_mat, P_img2cam[:3])
        imgpt2 = (img2_pt_h[:2] / img2_pt_h[2]).astype(int)

        # 3. I want P_tag14
        T_img2cam_tag14 = self.rs_tag14['transformation_matrix']
        T_tag14_img2cam = mr.TransInv(T_img2cam_tag14)
        P_tag14 = np.dot(T_tag14_img2cam, P_img2cam)
        print(f"P_tag14: {P_tag14}")

        # 4. I want P_img1cam
        T_img1cam_tag14 = self.rs_tag14_forclick['transformation_matrix']
        P_img1cam = np.dot(T_img1cam_tag14, P_tag14)
        print(f"P_img1cam: {P_img1cam}")

        img1_pt_h = np.dot(self.rs_camera_mat, P_img1cam[:3])
        imgpt = (img1_pt_h[:2] / img1_pt_h[2]).astype(int)
        print(f"Point in Image 1:{imgpt}")
        print(f"Point in Image 2:{imgpt2}")

        # imgpt = np.array([197, 182])
        # imgpt2 = np.array([323, 158])
        K = self.rs_camera_mat

        # Projection matrix for camera 1 (image1cam) (img1cam_world)
        T_img1cam_tag14 = self.rs_tag14_forclick['transformation_matrix']
        T_img2cam_tag14 = self.rs_tag14['transformation_matrix']
        T_tag14_img2cam = mr.TransInv(T_img2cam_tag14)
        T_img2cam_tag4 = self.rs_tag4['rotated_transformation_matrix']



        T_img1cam_img2cam = np.dot(T_img1cam_tag14, T_tag14_img2cam)
        T_img1cam_tag4 = np.dot(T_img1cam_img2cam, T_img2cam_tag4)
        T_img1cam_world = np.dot(T_img1cam_tag4, T_tag4_world)
        P1 = np.dot(K, T_img1cam_world[:3, :])  

        # Projection matrix for camera 2 (image2cam
        T_img2cam_world = np.dot(T_img2cam_tag4 , T_tag4_world) # You might need to combine with T_tag4_world if necessary
        P2 = np.dot(K, T_img2cam_world[:3, :])  # Projection matrix for image2cam

        # Convert 2D points to homogeneous coordinates
        imgpt1_homog = np.array([self.img1_pt[0], self.img1_pt[1], 1.0])
        imgpt2_homog = np.array([self.img2_pt[0], self.img2_pt[1], 1.0])

        # Triangulate to find the 3D point
        points_4d_homog = cv2.triangulatePoints(P1, P2, imgpt1_homog[:2], imgpt2_homog[:2])

        # Convert from homogeneous to 3D coordinates
        points_3d = points_4d_homog[:3] / points_4d_homog[3]

        print(f"Triangulated 3D Point: {points_3d}")

        reprojected_img1_homog = np.dot(P1, points_4d_homog)
        reprojected_img1 = (reprojected_img1_homog[:2] / reprojected_img1_homog[2]).astype(int)
        print(f"Reprojected Point in Image 1: {reprojected_img1}")

        reprojected_img2_homog = np.dot(P2, points_4d_homog)
        reprojected_img2 = (reprojected_img2_homog[:2] / reprojected_img2_homog[2]).astype(int)
        print(f"Reprojected Point in Image 2: {reprojected_img2}")







        








        # T_rs_tag14_forclick = np.array(self.rs_tag14_forclick["rotated_transformation_matrix"])
        # T_rs_tag14 = np.array(self.rs_tag14["rotated_transformation_matrix"])

        # # Projection Matrix
        # P_rs_tag14_forclick = np.dot(self.rs_camera_mat, T_rs_tag14_forclick[:3, :])
        # P_rs_tag14 = np.dot(self.rs_camera_mat, T_rs_tag14[:3, :])

        # # Homogenous 
        # Ph_rs_tag14_forclick = np.array([[self.img1_pt[0]], [self.img1_pt[1]]], dtype=np.float64)
        # Ph_rs_tag4 = np.array([[self.img2_pt[0]], [self.img2_pt[1]]], dtype=np.float64)

        # # Triangulate 
        # point_h = cv2.triangulatePoints(P_rs_tag14_forclick, P_rs_tag14, Ph_rs_tag14_forclick, Ph_rs_tag4)
        # point_3d_frame1 = point_h[:3] / point_h[3]

        # # Transform from frame 1 to frame 2 (tag4 is only visible in frame 2)
        # frame1_inv = mr.TransInv(T_rs_tag14_forclick)
        # R_frame1 = frame1_inv[:3, :3]
        # t_frame1 = frame1_inv[:3, 3]
        # R_frame2 = T_rs_tag14[:3, :3]
        # t_frame2 = T_rs_tag14[:3, 3]
        # R_t14c_t14 = np.dot(R_frame2, R_frame1)
        # t_t14c_t14 = (t_frame2 - np.dot(R_t14c_t14, t_frame1)).reshape(3, 1)
        # point_frame2 = (np.dot(R_t14c_t14, point_3d_frame1) + t_t14c_t14).reshape(3, 1)

        # # Transform relative to tag4
        # T_rs_tag4 = np.array(self.rs_tag4["rotated_transformation_matrix"])
        # R_rs_tag4 = T_rs_tag4[:3, :3]
        # t_rs_tag4 = (T_rs_tag4[:3, 3]).reshape(3, 1)
        # point_tag4 = np.dot(R_rs_tag4, (point_frame2 - t_rs_tag4))

        # print(f"Frame 1 - Triangulated Point: {point_3d_frame1}")
        # print(f"Frame 2 - Triangulated Point: {point_frame2}")
        # print(f"Tag4 - Triangulated Point: {point_tag4}")
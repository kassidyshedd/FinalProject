import os
import cv2
import apriltag
import numpy as np
import modern_robotics as mr
from robot.blob_detection_functions import BlobDetection 

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

        self.top_fid = None
        self.side_fid = None

        self.img1_pt = None
        self.img2_pt = None
        self.world_pt = None
        self.top_pt = None
        self.side_pt = None
        self.target_pt = None

        self.topjson = '/home/kashedd/finalproject/code/ros/ws_final/top.json'
        self.sidejson = '/home/kashedd/finalproject/code/ros/ws_final/side.json'


        self.camera_to_tag4 = None
        self.camera_to_tag14 = None
        self.world_to_tag4 = None

        self.T_world_tag4 = np.array([
            [1, 0, 0, 0], 
            [0, 1, 0, -0.1141],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])
        
        self.T_fid_tag14 = np.array([
            [1, 0, 0, 0.0375],
            [0, 0, 1, 0],
            [0, -1, 0, 0.0375],
            [0, 0, 0, 1]])
        
        self.T_top_side = np.array([
            [1, 0, 0, 0], 
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]])

        self.fid1_obj_pts = np.array([
                    [2.5, 2.5, 21], [7.5, 42.5, 21], [42.5, 37.5, 21], # Layer #1
                    [7.5, 22.5, 16], [17.5, 37.5, 16], [37.5, 32.5, 16], [32.5, 7.5, 16], # Layer #2
                    [12.5, 17.5, 11], [27.5, 32.5, 11], [32.5, 22.5, 11], [17.5, 12.5, 11], # Layer #3
                    [17.5, 17.5, 6], [27.5, 27.5, 6], [22.5, 22.5, 6]], dtype=np.float32)
        
        self.fid_obj_pts = np.array([
                    [2.5, 2.5, 21], [42.5, 37.5, 21], # Layer #1
                    [37.5, 32.5, 16], [32.5, 7.5, 16], # Layer #2
                    [12.5, 17.5, 11], [27.5, 32.5, 11], [32.5, 22.5, 11],  # Layer #3
                    [17.5, 17.5, 6], [27.5, 27.5, 6], [22.5, 22.5, 6]], dtype=np.float32)

    def get_rs_tag4(self):
        return self.rs_tag4
    
    def get_rs_tag14(self):
        return self.rs_tag14

    def get_point(self):
        return self.world_pt


    def open_images(self, photo_array):
        for index, path in enumerate(photo_array):
            if os.path.exists(path):
                image = cv2.imread(path)
                self.detect_tags(image, index, path)
                self.display_image_click_capture(image, path, index)

    
    def detect_tags(self, image, index, path):
        # First 2 photos will use apriltag library (taken with realsense) (if testing irl all 4 will use)
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
                    # print(self.rs_tag14_forclick['center'])
                # Livestream photo with tag4 and tag14
                if index == 1 and detection.tag_id == 4:
                    self.rs_tag4 = detection_info
                    # print(self.rs_tag4['center'])
                if index == 1 and detection.tag_id == 14:
                    self.rs_tag14 = detection_info
                    # print(self.rs_tag14['center'])
                self.draw_axes(image, detection.center, T_mat, r_T_mat)

        if index == 2:
            bdtop = BlobDetection(image_path=path, json_filename='top.json')
            bdtop.run()
            top_centers = bdtop.open_json(self.topjson)
            all_fid_obj_pts = []
            all_f1_pts = []
            for path, mc in top_centers:
                all_fid_obj_pts.append(self.fid1_obj_pts)
                all_f1_pts.append(np.array(mc, dtype=np.float32))

            # Concatenate all points into single numpy arrays
            all_fid_obj_pts = np.vstack(all_fid_obj_pts)
            all_f1_pts = np.vstack(all_f1_pts)

            ret, rvec, tvec = cv2.solvePnP(all_fid_obj_pts, all_f1_pts, self.top_camera_mat, self.top_dist_coeffs)
            top_T_mat = self.create_transformation_matrix(rvec, tvec)
            self.top_fid = {
                "rvec": rvec,
                "tvec": tvec,
                "transformation_matrix": top_T_mat}
            # print(self.top_fid)

        if index == 3:
            bdside = BlobDetection(image_path=path, json_filename='side.json')
            bdside.run()
            side_centers = bdside.open_json(self.sidejson)
            all_fid_obj_pts = []
            all_fs_pts = []
            for path, mc in side_centers:
                all_fid_obj_pts.append(self.fid_obj_pts)
                all_fs_pts.append(np.array(mc, dtype=np.float32))

            # Concatenate all points into single numpy arrays
            all_fid_obj_pts = np.vstack(all_fid_obj_pts)
            all_fs_pts = np.vstack(all_fs_pts)

            ret, rvec, tvec = cv2.solvePnP(all_fid_obj_pts, all_fs_pts, self.side_camera_mat, self.side_dist_coeffs)
            side_T_mat = self.create_transformation_matrix(rvec, tvec)
            self.side_fid = {
                "rvec": rvec,
                "tvec": tvec,
                "transformation_matrix": side_T_mat}
            # print(self.side_fid)


            
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
        

    def draw_axes(self, image, center, transform_matrix, r_transform_matrix):
        axis_len = 0.1
        axis_pts = np.float32([
            [axis_len, 0, 0],
            [0, axis_len, 0],
            [0, 0, axis_len],
            [0, 0, 0]]).reshape(-1, 3)
        
        center = tuple(map(int, center))
        
        # Original Coordiate Frame
        img_pts, _ = cv2.projectPoints(axis_pts, transform_matrix[:3, :3], transform_matrix[:3, 3], self.rs_camera_mat, self.rs_dist_coeffs)
        img_pts = img_pts.astype(int)
        cv2.line(image, center, tuple(img_pts[0].ravel()), (0, 0, 255), 3)  # X-axis in red
        cv2.line(image, center, tuple(img_pts[1].ravel()), (0, 255, 0), 3)  # Y-axis in green
        cv2.line(image, center, tuple(img_pts[2].ravel()), (255, 0, 0), 3)  # Z-axis in blue

        # Corrected Coordinate Frame (rotate 180 degress around x-axis)
        # r_img_pts, _ = cv2.projectPoints(axis_pts, r_transform_matrix[:3, :3], r_transform_matrix[:3, 3], self.rs_camera_mat, self.rs_dist_coeffs)
        # r_img_pts = r_img_pts.astype(int)
        # cv2.line(image, center, tuple(r_img_pts[0].ravel()), (0, 0, 200), 1, lineType=cv2.LINE_AA)  # X-axis in red (rotated)
        # cv2.line(image, center, tuple(r_img_pts[1].ravel()), (0, 200, 0), 1, lineType=cv2.LINE_AA)  # Y-axis in green (rotated)
        # cv2.line(image, center, tuple(r_img_pts[2].ravel()), (200, 0, 0), 1, lineType=cv2.LINE_AA)  # Z-axis in blue (rotated)

    
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
            if index == 2:
                self.top_pt = (x, y)
                print(f"Clicked Point in Image 3: {self.top_pt}")
            if index == 3:
                self.side_pt = (x, y)
                print(f"Clicked Point in Image 4: {self.side_pt}")

            if self.img1_pt and self.img2_pt is not None and self.top_pt and self.side_pt is not None:
                P_entry = self.triangulate_entry()
                P_target = self.triangulate_target()

                # x =  P_target[0] - P_entry[0] 
                # y =  P_target[1] - P_entry[1] 
                # z =  P_target[2] - P_entry[2] 

                # norm = np.sqrt(x**2 + y**2 + z**2)
                # dx = x / norm
                # dy = y / norm
                # dz = z / norm

                # # Calculate roll, pitch, and yaw
                # yaw = np.arctan2(dy, dx)  # Yaw around Z-axis
                # pitch = np.arctan2(-dz, np.sqrt(dx**2 + dy**2))  # Pitch around Y-axis
                
     
    def triangulate_entry(self):
        # Testing Transforms
        # 1. I want P_tag4 -- P_world = [0.1, 0.1, 0.1]
        P_world = np.array([0.35995023, -0.13087515, 0.25986143, 1])
        # print(f"P_world: {P_world}")
        T_tag4_world = mr.TransInv(self.T_world_tag4)
        P_tag4 = np.dot(T_tag4_world, P_world)
        # print(f"P_tag4: {P_tag4}")

        # 2. I want P_camera
        T_img2cam_tag4 = self.rs_tag4['rotated_transformation_matrix']
        # print(T_camera_tag4)
        P_img2cam = np.dot(T_img2cam_tag4, P_tag4)
        # print(f"P_camera: {P_img2cam}")
        img2_pt_h = np.dot(self.rs_camera_mat, P_img2cam[:3])
        imgpt2 = (img2_pt_h[:2] / img2_pt_h[2]).astype(int)

        # 3. I want P_tag14
        T_img2cam_tag14 = self.rs_tag14['transformation_matrix']
        T_tag14_img2cam = mr.TransInv(T_img2cam_tag14)
        P_tag14 = np.dot(T_tag14_img2cam, P_img2cam)
        # print(f"P_tag14: {P_tag14}")

        # 4. I want P_img1cam
        T_img1cam_tag14 = self.rs_tag14_forclick['transformation_matrix']
        P_img1cam = np.dot(T_img1cam_tag14, P_tag14)
        # print(f"P_img1cam: {P_img1cam}")

        img1_pt_h = np.dot(self.rs_camera_mat, P_img1cam[:3])
        imgpt = (img1_pt_h[:2] / img1_pt_h[2]).astype(int)
        # print(f"Point in Image 1:{imgpt}")
        # print(f"Point in Image 2:{imgpt2}")

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
        T_img2cam_world = np.dot(T_img2cam_tag4 , T_tag4_world) 
        P2 = np.dot(K, T_img2cam_world[:3, :])  # Projection matrix for image2cam

        # Convert 2D points to homogeneous coordinates
        imgpt1_homog = np.array([self.img1_pt[0], self.img1_pt[1], 1.0])
        imgpt2_homog = np.array([self.img2_pt[0], self.img2_pt[1], 1.0])
        # imgpt1_homog = np.array([imgpt[0], imgpt[1], 1.0])
        # imgpt2_homog = np.array([imgpt2[0], imgpt2[1], 1.0])

        # Triangulate to find the 3D point
        points_4d_homog = cv2.triangulatePoints(P1, P2, imgpt1_homog[:2], imgpt2_homog[:2])

        # Convert from homogeneous to 3D coordinates
        points_3d = points_4d_homog[:3] / points_4d_homog[3]
        self.world_pt = points_3d

        # print(f"Triangulated 3D Entry Point: {points_3d}")

        reprojected_img1_homog = np.dot(P1, points_4d_homog)
        reprojected_img1 = (reprojected_img1_homog[:2] / reprojected_img1_homog[2]).astype(int)
        # print(f"Reprojected Point in Image 1: {reprojected_img1}")

        reprojected_img2_homog = np.dot(P2, points_4d_homog)
        reprojected_img2 = (reprojected_img2_homog[:2] / reprojected_img2_homog[2]).astype(int)
        # print(f"Reprojected Point in Image 2: {reprojected_img2}")


    def triangulate_target(self):
        P_tag14 = [0, 0, 0, 1]
        P_fid = np.dot(self.T_fid_tag14, P_tag14)
        P_topcam = np.dot(self.top_fid['transformation_matrix'], P_fid)
        imgtop_pt_h = np.dot(self.top_camera_mat, P_topcam[:3])
        imgtop_pt= (imgtop_pt_h[:2] / imgtop_pt_h[2]).astype(int)
        print(f"Point in Top Camera: {imgtop_pt}")

        T_side_top = mr.TransInv(self.T_top_side)
        P_sidecam = np.dot(self.side_fid['transformation_matrix'], P_fid)
        P_sidecam = np.dot(T_side_top, P_sidecam)
        imgside_pt_h = np.dot(self.side_camera_mat, P_sidecam[:3])
        imgside_pt= (imgside_pt_h[:2] / imgside_pt_h[2]).astype(int)
        print(f"Point in Side Camera: {imgside_pt}")

        k = self.top_camera_mat
        T_top_fid = self.top_fid['transformation_matrix']
        T_top_tag14 = np.dot(T_top_fid, self.T_fid_tag14)
        P_top = np.dot(k, T_top_tag14[:3, :])

        T_side_fid = self.side_fid['transformation_matrix']
        T_side_tag14 = np.dot(T_side_fid, self.T_fid_tag14)
        P_side = np.dot(k, T_side_tag14[:3, :])

        imgpt1_homog = np.array([imgtop_pt[0], imgtop_pt[1], 1.0])
        imgpt2_homog = np.array([imgside_pt[0], imgside_pt[1], 1.0])

        points_4d_homog = cv2.triangulatePoints(P_top, P_side, imgpt1_homog[:2], imgpt2_homog[:2])

        points_3d = points_4d_homog[:3] / points_4d_homog[3]
        self.target_pt = points_3d
        P_tag14 = points_4d_homog
        print(f"Target Point: {P_tag14}")

        # P_img1cam
        T_img1cam_tag14 = self.rs_tag14['transformation_matrix']
        P_img1cam = np.dot(T_img1cam_tag14, P_tag14)

        # P_tag4
        T_tag4_img1cam = mr.TransInv(self.rs_tag4['transformation_matrix'])
        P_tag4 = np.dot(T_tag4_img1cam, P_img1cam)

        # P_world
        P_world = np.dot(self.T_world_tag4, P_tag4)
        print(f"Target Point in world coords: {P_world}")

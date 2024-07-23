# Imports
import pyrealsense2 as rs
import numpy as np
import cv2
import apriltag



class ModelRobot:

    def __init__(self, image_directory, flag=False):

        # Initilize variables
        self.image_directory = image_directory
        self.flag = flag
        self.tags_detected = False
        print('Variables Initialized')

        # Start livestream camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        print("Camera Stream Started")


    def start_livestream(self):
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not  color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())

                self.tags_info = self.detect_aprilTag_livestream(color_image)
                if self.tags_info is not None:
                    if self.tags_detected == False:
                        self.tags_detected = True
                        print(f"Detected Tags: {self.tags_info}")
                        print("Tags Detected!")
                        

                cv2.namedWindow("Realsense Stream", cv2.WINDOW_AUTOSIZE)
                cv2.imshow("Realsense", color_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

    def calibrateCamera():
        intrinsics = None
        return intrinsics

    def detect_aprilTag_livestream(self, frame):

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Initialize AprilTag detector
        options = apriltag.DetectorOptions(families="tag16h5")
        detector = apriltag.Detector(options)
        tags = detector.detect(gray)

        tags_info = {}

        # Store tag informaton
        for detections in tags:
            if detections.tag_id in [14, 19]:
                tags_info[detections.tag_id] = {
                    'corners': detections.corners,
                    'center': detections.center
                }

                for corner in detections.corners:
                    corner = (int(corner[0]), int(corner[1]))
                    cv2.circle(frame, corner, 5, (0, 255, 0), 2)

        if len(tags_info) == 2:
            return tags_info
        else:
            return None
    

    def get_livestream_cam_pose(self):

        tag_size = 4.5 # cm
        obj_pts = np.array([
            [-tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, -tag_size / 2, 0], 
            [tag_size / 2, tag_size / 2, 0],
            [-tag_size / 2, tag_size / 2, 0]
        ], dtype=np.float32)

        img_pts = np.array(self.tags_info[14]['corners'], dtype=np.float32)

        # Camera Matrix (Eventually not hardcoded)
        camera_mat = np.array([[606.6510009765625, 0, 321.1846923828125], 
                              [0, 606.622802734375, 246.59637451171875], 
                              [0, 0, 1]], dtype=np.float32)
        
        distort_coef = np.zeros(5)

        _, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_mat, distort_coef)

        pose = self.transformation_matrix(rvec, tvec)

        return pose
    
    def transformation_matrix(self, rvec, tvec):
        rotation_mat, _ = cv2.Rodrigues(rvec)
        
        tmat = np.eye(4)
        tmat[:3, :3] = rotation_mat
        tmat[:3, 3] = tvec.flatten()

        return tmat
    
    def detect_AprilTag_image(self, image):

        image = cv2.imread(image)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        options = apriltag.DetectorOptions(families="tag16h5")
        detector = apriltag.Detector(options)
        tags = detector.detect(gray)

        top_image = {}

        for detections in tags:
            id_ = detections.tag_id
            top_image[detections.tag_id] = {
                'corners': detections.corners,
                'center': detections.center}
            
        return id_, top_image
    
    def get_pose_from_image(self, id_, dict):

        tag_size = 4.5 # cm
        obj_pts = np.array([
            [-tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, -tag_size / 2, 0], 
            [tag_size / 2, tag_size / 2, 0],
            [-tag_size / 2, tag_size / 2, 0]
        ], dtype=np.float32)

        img_pts = np.array(dict[id_]['corners'], dtype=np.float32)

        # Camera Matrix (Eventually not hardcoded)
        camera_mat = np.array([[606.6510009765625, 0, 321.1846923828125], 
                              [0, 606.622802734375, 246.59637451171875], 
                              [0, 0, 1]], dtype=np.float32)
        
        distort_coef = np.zeros(5)

        _, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_mat, distort_coef)

        pose = self.transformation_matrix(rvec, tvec)

        return pose




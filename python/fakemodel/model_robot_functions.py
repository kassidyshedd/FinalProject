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


        

    def getPose():
        pass






    
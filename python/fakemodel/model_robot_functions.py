# Imports
import pyrealsense2 as rs
import numpy as np
import cv2



class ModelRobot:

    def __init__(self, image_directory, flag=False):

        # Initilize variables
        self.image_directory = image_directory
        self.flag = flag
        print('Variables Initialized')

        # Start livestream camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)
        print("Camera Stream Started")


        # Need to save position of apriltag on fiducial
        # Need to save position of pariltag on robot

    def startLivestream(self):
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not  color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())

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

    def detectAprilTag():
        pass

    def getPose():
        pass






    
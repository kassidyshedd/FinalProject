# Functions for image processing

import cv2
import numpy as np
import os
import json

class ImageProcessing:

    """ Add information about function"""
    def __init__(self, image_directory):

        self.threshold_v = 100
        self.blur_kernel_size = 11
        self.min_area = 30
        self.max_area = 5000
        self.min_circularity = 70
        self.min_inertia_ratio = 70
        self.min_convexity = 70

        self.image_directory = image_directory
        self.image_files = [f for f in os.listdir(self.image_directory)]
        self.image_index = 0

        self.annotated_points = []
        self.marker_centers_list = []

    """ Add information about function """
    def on_trackbar_change(self, value):
        pass

    """ Add information about function"""
    def setup_trackbars(self, window_name):
        cv2.createTrackbar('Threshold', window_name, self.threshold_value, 255, self.on_trackbar_change)
        cv2.createTrackbar('Blur Kernel', window_name, self.blur_kernel_size, 50, self.on_trackbar_change)
        cv2.createTrackbar('Minimum Area', window_name, self.min_area, 5000, self.on_trackbar_change)
        cv2.createTrackbar('Maximum Area', window_name, self.max_area, 10000, self.on_trackbar_change)
        cv2.createTrackbar('Minimum Circularity', window_name, self.min_circularity, 100, self.on_trackbar_change)
        cv2.createTrackbar('Minimum Inertia Ratio', window_name, self.min_inertia_ratio, 100, self.on_trackbar_change)
        cv2.createTrackbar('Minimum Convexity', window_name, self.min_convexity, 100, self.on_trackbar_change)

    """ Add information about function"""
    def get_trackbar_values(self, window_name):
        threshold = cv2.getTrackbarPos('Threshold', window_name)
        blur_kernel = cv2.getTrackbarPos('Blur Kernel', window_name)
        min_area =  cv2.getTrackbarPos('Minimum Area', window_name)
        max_area = cv2.getTrackbarPos('Maximum Area', window_name)
        min_circularity = cv2.getTrackbarPos('Minimum Circularity', window_name) / 100.0
        min_inertia_ratio = cv2.getTrackbarPos('Minimum Inertia Ratio', window_name) / 100.0
        min_convexity = cv2.getTrackbarPos('Minimum Convexity', window_name) / 100.0

        return threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity
    
    """ Add information about function"""
    def process_image_png(self, image, threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity):
        gs = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gs, (blur_kernel * 2 + 1, blur_kernel * 2 + 1), 0)
        _, binary = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY)

        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = min_area
        params.maxArea = max_area
        params.filterByCircularity = True
        params.minCircularity = min_circularity
        params.filterByInertia = True
        params.minInertiaRatio = min_inertia_ratio
        params.filterByConvexity = True
        params.minConvexity = min_convexity

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(binary)
        
        image_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return image_with_keypoints, keypoints
    
    """ Add information about function"""
    def display_blob_count(self, image, keypoints):
        count = f'Blob Count: {len(keypoints)}'
        cv2.putText(image, count, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    """ Add information about function"""
    def on_mouse(self, event, x, y, flags, param):
        image, keypoints = param
        if event == cv2.EVENT_LBUTTONDOWN:
            closest_dist = float("inf")
            closest_point = None
            for kp in keypoints:
                kpx, kpy = kp.pt
                dist = np.sqrt((kpx - x)**2 + (kpy - y)**2)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_point = (int(kpx), int(kpy))
                if closest_point:
                    self.annotated_points.append(closest_point)
                    cv2.circle(image, closest_point, 5, (0, 255, 0), -1)
                    cv2.imshow('Annotate Blobs', image)

    """ Add information about function"""
    def save_marker_centers(self):
        with open('marker_centers.json', 'w') as f:
            json.dump(self.marker_centers_list, f)
        print("Marker Centers Saved.")

    """ Add information about function"""
    def run_blob_detection(self):
        window_name = 'Blob Detection'
        cv2.namedWindow(window_name)
        self.setup_trackbars(window_name)

        while self.image_index < len(self.image_files):
            image_path = os.path.join(self.image_directory, self.image_files[self.image_index])
            image = cv2.imread(image_path)

            while True:
                threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity = self.get_trackbar_values(window_name)
                processed_image, keypoints = self.process_image_png(image, threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity)
                self.display_blob_count(processed_image, keypoints)
                cv2.imshow(window_name, processed_image)

                key = cv2.waitKey(1) & 0xFF
                if key == 13:
                    cv2.imshow('Annotate Blobs', processed_image)
                    cv2.setMouseCallback('Annotate Blobs', self.on_mouse, (processed_image, keypoints))

                    while True:
                        if cv2.waitKey(1) & 0xFF == 13:
                            break
                    cv2.destroyWindow('Annotate Blobs')

                    if len(self.annotated_points) == len(keypoints):
                        self.marker_centers_list.append((image_path, self.annotated_points.copy()))
                        print(f"Saved markers for {image_path}")
                        self.annotated_points.clear()
                        self.image_index += 1
                        break
                    else:
                        print("Incorrect number of points.")
                        self.annotated_points.clear()
                
                elif key == ord('d'):
                    print(f"Deleted {image_path}")
                    os.remove(image_path)
                    self.image_index += 1
                    break

        cv2.destroyAllWindows()
        self.save_marker_centers()


    



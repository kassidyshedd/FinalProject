import cv2
import numpy as np
import os
import json

class BlobDetection:
    def __init__(self, image_dir):

        # Initialize slider values
        self.threshold_value = 100
        self.blur_kernel_size = 11
        self.min_area = 30
        self.max_area = 5000
        self.min_circularity = 70
        self.min_inertia_ratio = 70
        self.min_convexity = 70

        self.marker_center_list = []
        self.annotated_points = []

        self.image_dir = image_dir
        self.image_files = [f for f in os.listdir(image_dir) if f.endswith('.png')]
        self.image_index = 0

        self.window_name = 'Blob Detection'
        cv2.namedWindow(self.window_name)
        self.setup_trackbars()

    
    def get_image_size(self, image_path):
        image = cv2.imread(image_path)
        height, width, channels = image.shape
        return width, height, channels
    

    
    def on_trackbar_change(self, val):
        pass

    
    def setup_trackbars(self):
        cv2.createTrackbar('Threshold', self.window_name, self.threshold_value, 255, self.on_trackbar_change)
        cv2.createTrackbar('Blur Kernel', self.window_name, self.blur_kernel_size, 50, self.on_trackbar_change)
        cv2.createTrackbar('Min Area', self.window_name, self.min_area, 5000, self.on_trackbar_change)
        cv2.createTrackbar('Max Area', self.window_name, self.max_area, 10000, self.on_trackbar_change)
        cv2.createTrackbar('Min Circularity', self.window_name, self.min_circularity, 100, self.on_trackbar_change)
        cv2.createTrackbar('Min Inertia Ratio', self.window_name, self.min_inertia_ratio, 100, self.on_trackbar_change)
        cv2.createTrackbar('Min Convexity', self.window_name, self.min_convexity, 100, self.on_trackbar_change)


    def get_trackbar_values(self):
        threshold = cv2.getTrackbarPos('Threshold', self.window_name)
        blur_kernel = cv2.getTrackbarPos('Blur Kernel', self.window_name)
        min_area = cv2.getTrackbarPos('Min Area', self.window_name)
        max_area = cv2.getTrackbarPos('Max Area', self.window_name)
        min_circularity = cv2.getTrackbarPos('Min Circularity', self.window_name) / 100.0
        min_inertia_ratio = cv2.getTrackbarPos('Min Inertia Ratio', self.window_name) / 100.0
        min_convexity = cv2.getTrackbarPos('Min Convexity', self.window_name) / 100.0
        return threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity
    

    def process_image(self, image, threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity):
        gray_scale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray_scale, (blur_kernel * 2 + 1, blur_kernel * 2 + 1), 0)
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

        img_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return img_with_keypoints, keypoints
    

    def display_blob_count(self, image, keypoints, scaling_factor):
        for kp in keypoints:
            # Scale coordinates to match resized image
            x = int(kp.pt[0] * scaling_factor)
            y = int(kp.pt[1] * scaling_factor)
            cv2.circle(image, (x, y), 5, (0, 0, 255), -1)  # Red circle
        count_text = f'Blob Count: {len(keypoints)}'
        cv2.putText(image, count_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)


    def on_mouse(self, event, x, y, flags, param):
        image, keypoints, scaling_factor = param
        if event == cv2.EVENT_LBUTTONDOWN:
            closest_dist = float('inf')
            closest_pt = None
            for kp in keypoints:
                kp_x, kp_y = kp.pt
                # Scale coordinates to match resized image
                kp_x *= scaling_factor
                kp_y *= scaling_factor
                dist = np.sqrt((kp_x - x) ** 2 + (kp_y - y) ** 2)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_pt = (int(kp_x), int(kp_y))
            if closest_pt:
                self.annotated_points.append((int(closest_pt[0] / scaling_factor), int(closest_pt[1] / scaling_factor)))  # Save original coordinates
                cv2.circle(image, closest_pt, 5, (0, 255, 0), -1)
                cv2.imshow('Annotate Blobs', image)

    
    def save_marker_centers(self):
        with open('marker_centers.json', 'w') as f:  
            json.dump(self.marker_center_list, f) 


    def resize_image(self, image, max_width, max_height):
        height, width = image.shape[:2]
        scaling_factor = 1
        if width > max_width or height > max_height:
            scaling_factor = min(max_width / width, max_height / height)
            image = cv2.resize(image, (int(width * scaling_factor), int(height * scaling_factor)))
        return image, scaling_factor
    

    def open_json(self, name):
        with open(name, 'r') as f:
            marker_centers = json.load(f)
        return marker_centers


    def run(self):
        max_width = 800  # Set the max width for your display window
        max_height = 600  # Set the max height for your display window

        while self.image_index < len(self.image_files):
            image_path = os.path.join(self.image_dir, self.image_files[self.image_index])
            image = cv2.imread(image_path)
            
            while True:
                threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity = self.get_trackbar_values()
                processed_image, keypoints = self.process_image(image, threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity)
                
                # Resize image and get scaling factor
                resized_image, scaling_factor = self.resize_image(processed_image, max_width, max_height)
                self.display_blob_count(resized_image, keypoints, scaling_factor)
                cv2.imshow(self.window_name, resized_image)
                
                key = cv2.waitKey(1) & 0xFF
                if key == 13:  # Enter key
                    cv2.imshow('Annotate Blobs', resized_image)
                    cv2.setMouseCallback('Annotate Blobs', self.on_mouse, (resized_image, keypoints, scaling_factor))
                    
                    print("Click on each detected blob in the correct order. Press 'Enter' when done.")
                    while True:
                        key = cv2.waitKey(1) & 0xFF
                        if key == 13:  # Enter key
                            break
                    
                    cv2.destroyWindow('Annotate Blobs')

                    if len(self.annotated_points) == len(keypoints):
                        self.marker_center_list.append((image_path, self.annotated_points.copy()))
                        print(f"Saved marker centers for {image_path}")
                        self.annotated_points.clear()
                        self.image_index += 1
                        break
                    else:
                        print(f"Incorrect number of points annotated for image: {image_path}. Please re-annotate.")
                        self.annotated_points.clear()
                elif key == ord('d'):  # 'd' key for delete
                    print(f"Deleted {image_path}")
                    os.remove(image_path)
                    self.image_index += 1
                    break

        cv2.destroyAllWindows()
        self.save_marker_centers()



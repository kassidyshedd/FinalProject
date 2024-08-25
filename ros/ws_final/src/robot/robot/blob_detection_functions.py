import cv2
import numpy as np
import os
import json

"""
A class for detecting blobs in images, annotating them, and saving the coordinates of the annotated points.

Attributes:
-----------
image_dir : str
    Directory containing images to be processed. Either `image_dir` or `image_path` must be provided.
image_path : str
    Path to a specific image to be processed. Either `image_dir` or `image_path` must be provided.
json_filename : str
    Name of the JSON file where the annotated blob center coordinates will be saved.
threshold_value : int
    Initial threshold value for image processing.
blur_kernel_size : int
    Initial blur kernel size for image processing.
min_area : int
    Initial minimum area for blob detection.
max_area : int
    Initial maximum area for blob detection.
min_circularity : int
    Initial minimum circularity value for blob detection.
min_inertia_ratio : int
    Initial minimum inertia ratio for blob detection.
min_convexity : int
    Initial minimum convexity value for blob detection.
marker_center_list : list
    List of annotated marker centers.
annotated_points : list
    List of points annotated in the current image.
point_counter : int
    Counter for the number of points annotated in the current image.
image_files : list
    List of image filenames to be processed.
image_index : int
    Index of the current image being processed.
window_name : str
    Name of the OpenCV window used for displaying images.
"""


class BlobDetection:
    def __init__(self, image_dir=None, image_path=None, json_filename='marker_centers.json'):
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
        self.point_counter = 0  

        self.image_dir = image_dir
        self.image_path = image_path
        self.json_filename = json_filename

        if self.image_dir:
            self.image_files = [f for f in os.listdir(image_dir) if f.endswith('.png')]
        elif isinstance(self.image_path, str) and os.path.isfile(self.image_path):
            self.image_files = [os.path.basename(self.image_path)]
            self.image_dir = os.path.dirname(self.image_path)
        else:
            raise ValueError("Either image_dir or a valid image_path must be provided.")

        self.image_index = 0

        self.window_name = 'Blob Detection'
        cv2.namedWindow(self.window_name)
        self.setup_trackbars()


    """
    Gets the dimensions of the image at the specified path.

    Parameters:
    -----------
    image_path : str
        Path to the image file.

    Returns:
    --------
    width : int
        Width of the image.
    height : int
        Height of the image.
    channels : int
        Number of channels in the image.
    """
    def get_image_size(self, image_path):
        image = cv2.imread(image_path)
        height, width, channels = image.shape
        return width, height, channels


    """
    Callback function for trackbar changes.

    Parameters:
    -----------
    val : int
        Current value of the trackbar.
    """
    def on_trackbar_change(self, val):
        pass


    """
    Sets up trackbars in the OpenCV window for adjusting blob detection parameters.
    """
    def setup_trackbars(self):
        cv2.createTrackbar('Threshold', self.window_name, self.threshold_value, 255, self.on_trackbar_change)
        cv2.createTrackbar('Blur Kernel', self.window_name, self.blur_kernel_size, 50, self.on_trackbar_change)
        cv2.createTrackbar('Min Area', self.window_name, self.min_area, 5000, self.on_trackbar_change)
        cv2.createTrackbar('Max Area', self.window_name, self.max_area, 10000, self.on_trackbar_change)
        cv2.createTrackbar('Min Circularity', self.window_name, self.min_circularity, 100, self.on_trackbar_change)
        cv2.createTrackbar('Min Inertia Ratio', self.window_name, self.min_inertia_ratio, 100, self.on_trackbar_change)
        cv2.createTrackbar('Min Convexity', self.window_name, self.min_convexity, 100, self.on_trackbar_change)


    """
    Retrieves the current values of the trackbars.

    Returns:
    --------
    tuple
        A tuple containing the current values of the threshold, blur kernel, min area, max area, 
        min circularity, min inertia ratio, and min convexity trackbars.
    """
    def get_trackbar_values(self):
        threshold = cv2.getTrackbarPos('Threshold', self.window_name)
        blur_kernel = cv2.getTrackbarPos('Blur Kernel', self.window_name)
        min_area = cv2.getTrackbarPos('Min Area', self.window_name)
        max_area = cv2.getTrackbarPos('Max Area', self.window_name)
        min_circularity = cv2.getTrackbarPos('Min Circularity', self.window_name) / 100.0
        min_inertia_ratio = cv2.getTrackbarPos('Min Inertia Ratio', self.window_name) / 100.0
        min_convexity = cv2.getTrackbarPos('Min Convexity', self.window_name) / 100.0
        return threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity


    """
    Processes the image to detect blobs based on the specified parameters.

    Parameters:
    -----------
    image : ndarray
        The input image to process.
    threshold : int
        Threshold value for binary conversion.
    blur_kernel : int
        Kernel size for Gaussian blur.
    min_area : int
        Minimum area of blobs to detect.
    max_area : int
        Maximum area of blobs to detect.
    min_circularity : float
        Minimum circularity of blobs to detect.
    min_inertia_ratio : float
        Minimum inertia ratio of blobs to detect.
    min_convexity : float
        Minimum convexity of blobs to detect.

    Returns:
    --------
    img_with_keypoints : ndarray
        The input image with detected keypoints drawn on it.
    keypoints : list
        List of detected keypoints (blobs).
    """
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


    """
    Displays the count of detected blobs on the image.

    Parameters:
    -----------
    image : ndarray
        The image on which to display the blob count.
    keypoints : list
        List of detected keypoints (blobs).
    scaling_factor : float
        Scaling factor for resizing the image.
    """
    def display_blob_count(self, image, keypoints, scaling_factor):
        for kp in keypoints:
            x = int(kp.pt[0] * scaling_factor)
            y = int(kp.pt[1] * scaling_factor)
            cv2.circle(image, (x, y), 5, (0, 0, 255), -1) 
        count_text = f'Blob Count: {len(keypoints)}'
        cv2.putText(image, count_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)


    """
    Mouse callback function for annotating detected blobs in the image.

    Parameters:
    -----------
    event : int
        Type of mouse event.
    x : int
        X-coordinate of the mouse event.
    y : int
        Y-coordinate of the mouse event.
    flags : int
        Flags associated with the mouse event.
    param : tuple
        Additional parameters, including the image, keypoints, and scaling factor.
    """
    def on_mouse(self, event, x, y, flags, param):
        image, keypoints, scaling_factor = param
        if event == cv2.EVENT_LBUTTONDOWN:
            closest_dist = float('inf')
            closest_pt = None
            for kp in keypoints:
                kp_x, kp_y = kp.pt
                kp_x *= scaling_factor
                kp_y *= scaling_factor
                dist = np.sqrt((kp_x - x) ** 2 + (kp_y - y) ** 2)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_pt = (int(kp_x), int(kp_y))
            if closest_pt:
                self.point_counter += 1 
                self.annotated_points.append((int(closest_pt[0] / scaling_factor), int(closest_pt[1] / scaling_factor)))  # Save original coordinates
                cv2.circle(image, closest_pt, 5, (255, 0, 0), -1)
                cv2.putText(image, str(self.point_counter), (closest_pt[0] + 10, closest_pt[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                cv2.imshow('Annotate Blobs', image)


    """
    Saves the list of annotated marker centers to a JSON file.
    """
    def save_marker_centers(self):
        with open(self.json_filename, 'w') as f:
            json.dump(self.marker_center_list, f)


    """
    Resizes the image to fit within the specified maximum width and height, maintaining aspect ratio.

    Parameters:
    -----------
    image : ndarray
        The image to resize.
    max_width : int
        Maximum allowed width for the resized image.
    max_height : int
        Maximum allowed height for the resized image.

    Returns:
    --------
    image : ndarray
        The resized image.
    scaling_factor : float
        The scaling factor used for resizing.
    """
    def resize_image(self, image, max_width, max_height):
        height, width = image.shape[:2]
        scaling_factor = 1
        if width > max_width or height > max_height:
            scaling_factor = min(max_width / width, max_height / height)
            image = cv2.resize(image, (int(width * scaling_factor), int(height * scaling_factor)))
        return image, scaling_factor


    """
    Opens and reads the specified JSON file containing marker centers.

    Parameters:
    -----------
    name : str
        Name of the JSON file to open.

    Returns:
    --------
    marker_centers : list
        List of marker centers read from the JSON file.
    """
    def open_json(self, name):
        with open(name, 'r') as f:
            marker_centers = json.load(f)
        return marker_centers


    """
    Runs the blob detection process, allowing the user to interactively process and annotate images.

    The process continues until all images in the directory or specified image are processed. Annotated points are saved
    to a JSON file.
    """
    def run(self):
        max_width = 800  
        max_height = 600  

        while self.image_index < len(self.image_files):
            image_path = os.path.join(self.image_dir, self.image_files[self.image_index])
            image = cv2.imread(image_path)

            while True:
                threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity = self.get_trackbar_values()
                processed_image, keypoints = self.process_image(image, threshold, blur_kernel, min_area, max_area, min_circularity, min_inertia_ratio, min_convexity)

                resized_image, scaling_factor = self.resize_image(processed_image, max_width, max_height)
                self.display_blob_count(resized_image, keypoints, scaling_factor)
                cv2.imshow(self.window_name, resized_image)

                key = cv2.waitKey(1) & 0xFF
                if key == 13:  # Enter key
                    cv2.imshow('Annotate Blobs', resized_image)
                    cv2.setMouseCallback('Annotate Blobs', self.on_mouse, (resized_image, keypoints, scaling_factor))
                    self.point_counter = 0  

                    print("Click on each detected blob in the correct order. Press 'Enter' when done.")
                    while True:
                        key = cv2.waitKey(1) & 0xFF
                        if key == 13:  
                            break

                    cv2.destroyWindow('Annotate Blobs')

                    if len(self.annotated_points) == len(keypoints):
                        self.marker_center_list.append((image_path, self.annotated_points.copy()))
                        print(f"Saved marker centers for {image_path}")
                        self.annotated_points.clear()
                        self.image_index += 1
                        break
                    else:
                        print(f"Incorrect number of points annotated for image: {image_path}. Re-annotate.")
                        self.annotated_points.clear()
                elif key == ord('d'):  # 'd' key for delete
                    print(f"Deleted {image_path}")
                    os.remove(image_path)
                    self.image_index += 1
                    break

        cv2.destroyAllWindows()
        self.save_marker_centers()

from image_processing import ImageProcessing
import numpy as np
import cv2


INTRINSICS_NEEDED = True

# ########## FOR INTRINSIC CALIBRATION ONLY -- SET INTRINSICS_NEEDED = TRUE ##########
# Directory for calibration images
calib_image_dir = '/home/kashedd/finalproject/calibration_images'

# ##### Perform image calibration for camera intrinsics #####
image_processor = ImageProcessing(calib_image_dir)
image_processor.run_blob_detection()

# ##### Get Camera intrinsics #####
# Object Points from fiducial
obj_pts = np.array([
    [5, 5, 25],   [5, 45, 25],  [5, 85, 25],  [45, 85, 25], 
    [85, 85, 25], [85, 45, 25], [85, 5, 25],  [15, 15, 20], 
    [15, 35, 20], [15, 55, 20], [15, 75, 20], [35, 75, 20], 
    [55, 75, 20], [75, 75, 20], [75, 55, 20], [75, 35, 20], 
    [75, 15, 20], [55, 15, 20], [35, 15, 20], [25, 25, 15], 
    [25, 45, 15], [25, 65, 15], [45, 65, 15], [65, 65, 15], 
    [65, 45, 15], [65, 25, 15], [45, 25, 15], [35, 35, 10], 
    [35, 55, 10], [55, 55, 10], [55, 35, 10], [45, 45, 10]
], dtype=np.float32)

all_obj_pts = []
all_img_pts = []

marker_centers = image_processor.open_json('marker_centers.json')

for path, mc in marker_centers:
    all_obj_pts.append(obj_pts)
    all_img_pts.append(np.array(mc, dtype=np.float32))


img_size = (640, 480)

# Initial intrinsics guess
focal_len = img_size[0]
c = (img_size[0] / 2, img_size[1] / 2)

guess_cm = np.array([
    [focal_len, 0, c[0]], 
    [0, focal_len, c[1]],
    [0, 0, 1]
], dtype=np.float32)

guess_dc = np.zeros(5)

try:
    ret, camera_mat, dist_coeffs, rvec, tvec = cv2.calibrateCamera(all_obj_pts, all_img_pts, img_size, guess_cm, guess_dc, None, None, cv2.CALIB_USE_INTRINSIC_GUESS)
    print(f"Camera Matrix:\n{camera_mat}")
    print(f"Distortion Coefficients:\n{dist_coeffs}")
except cv2.error as e:
    print("Calibration Failed:", e)

# ####################################################################################
# Get pose information from top view image

# Get pose information from side view image

# Get pose information from apriltag on fiducial

# Get pose information from april tag on robot


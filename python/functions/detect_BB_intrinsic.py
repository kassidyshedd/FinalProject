from image_processing import ImageProcessing
import numpy as np
import cv2


INTRINSICS_NEEDED = True
camera_poses = []
camera_orientations = []

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


ret, camera_mat, dist_coeffs, rvec, tvec = cv2.calibrateCamera(all_obj_pts, all_img_pts, img_size, guess_cm, guess_dc, None, None, cv2.CALIB_USE_INTRINSIC_GUESS)
print(f"Camera Matrix:\n{camera_mat}")
print(f"Distortion Coefficients:\n{dist_coeffs}")

# ####################################################################################
# Get pose information from top view image
top_image = '/home/kashedd/finalproject/pose_images/Top'
process_top = ImageProcessing(top_image, flag=True)
top_centers = process_top.run_blob_detection()
top_centers = np.array(top_centers[0][1], dtype=np.float32).reshape(-1, 1, 2)

t_ret, t_rvec, t_tvec = cv2.solvePnP(obj_pts, top_centers, camera_mat, dist_coeffs)

t_distance = np.linalg.norm(t_tvec)
t_img_pts, _ = cv2.projectPoints(obj_pts, t_rvec, t_tvec, camera_mat, dist_coeffs)
t_reproj_error = np.sqrt(np.mean(np.sum((top_centers - t_img_pts.squeeze())**2, axis=1)))

print("Top Rotation Vector:\n", t_rvec)
print("Top Translation Vector:\n", t_tvec)
print("Top Distance from camera to origin (m):", t_distance)
print("Top Reprojection Error (pixels):", t_reproj_error)

# Find Top Camera tranformation in world coordinates
t_R, _ = cv2.Rodrigues(t_rvec)
t_transform = np.eye(4)
t_transform[:3, :3] = t_R
t_transform[:3, 3] = t_tvec.flatten()

t_transform_inv = np.linalg.inv(t_transform)
t_camera_pose = t_transform_inv[:3, 3]
camera_poses.append(t_camera_pose)
t_camera_orientation = t_transform_inv[:3, :3]
camera_orientations.append(t_camera_orientation)

print("Top Camera Position (world): ", t_camera_pose)
print("Top Camera Orientation (world): ", t_camera_orientation)


# Get pose information from side view image
side_image = '/home/kashedd/finalproject/pose_images/Side'
process_side = ImageProcessing(side_image, flag=True)
side_centers = process_side.run_blob_detection()
side_centers = np.array(side_centers[0][1], dtype=np.float32).reshape(-1, 1, 2)

s_ret, s_rvec, s_tvec = cv2.solvePnP(obj_pts, side_centers, camera_mat, dist_coeffs)

s_distance = np.linalg.norm(s_tvec)
s_img_pts, _ = cv2.projectPoints(obj_pts, s_rvec, s_tvec, camera_mat, dist_coeffs)
s_reproj_error = np.sqrt(np.mean(np.sum((side_centers - s_img_pts.squeeze())**2, axis=1)))

print("Side Rotation Vector:\n", s_rvec)
print("Side Translation Vector:\n", s_tvec)
print("Side Distance from camera to origin (m):", s_distance)
print("Side Reprojection Error (pixels):", s_reproj_error)

# Find Side Camera tranformation in world coordinates
s_R, _ = cv2.Rodrigues(s_rvec)
s_transform = np.eye(4)
s_transform[:3, :3] = s_R
s_transform[:3, 3] = s_tvec.flatten()

s_transform_inv = np.linalg.inv(s_transform)
s_camera_pose = s_transform_inv[:3, 3]
camera_poses.append(s_camera_pose)
s_camera_orientation = s_transform_inv[:3, :3]
camera_orientations.append(s_camera_orientation)

print("Side Camera Position (world): ", s_camera_pose)
print("Side Camera Orientation (world): ", s_camera_orientation)

# Get pose information from apriltag on fiducial

# Get pose information from april tag on robot

# Visualization
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

xp = [pos[0] for pos in camera_poses]
yp = [pos[1] for pos in camera_poses]
zp = [pos[2] for pos in camera_poses]

c = ['r', 'b']
markers = ['o', 'x']

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for i, (pos, ori) in enumerate(zip(camera_poses, camera_orientations)):
    x, y, z = pos
    dx, dy, dz = ori

    length = np.linalg.norm([dx, dy, dz])
    dx /= length
    dy /= length
    dz /= length

    ax.scatter(x, y, z, color=c[i], marker=markers[i], s=100, label=f'camera{i+1}')
    ax.quiver(x, y, z, dx, dy, dz, length=10, color=c[i])

    plt.show()




ax.scatter(xp, yp, zp, c='r', marker='o')

plt.show()


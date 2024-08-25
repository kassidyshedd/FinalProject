# Low-Cost Robot-Assisted Spinal Surgery Solution

The Low-Cost Robot-Assisted Spinal Surgery Solution package provides tools for calibrating cameras and controlling robotic movements based on 3D transformations. The system supports various cameras, including Intel RealSense and C-Arm devices, and utilizes AprilTags and custom blob detection algorithms for precise calibration and movement. 

## Core Features

- **Camera Calibration**: Utilizes AprilTags and blob detection to calibrate camera intrinsics, ensuring accurate 3D transformations.
- **3D Transformation Calculation**: Computes precise 3D transformations between different frames, allowing for accurate robot movements.
- **Robot Control**: Commands a robot to move based on the triangulated 3D points from calibrated camera images.
- **Fiducial and AprilTag Detection**: Detects fiducials and AprilTags in images, estimating their poses for accurate spatial localization.


## Nodes

### `move_robot_calculations`

The `move_robot_calculations` node contains the logic for calculating robot movements based on 3D transformations and detected fiducials. It includes:

- **MoveRobotCalculations**: Handles the computation of transformations and the triangulation of points in 3D space.
- **Triangulation and Pose Estimation**: Uses inputs from multiple camera views to triangulate points and estimate poses.

### `camera_calibration`

The `camera_calibration` node is responsible for calibrating the camera intrinsics using blob detection. It includes:

- **CameraCalibration**: Handles the calibration process, saving intrinsics to YAML files for later use.
- **AprilTag Detection and Blob Detection**: Uses custom blob detection algorithms to determine camera parameters.




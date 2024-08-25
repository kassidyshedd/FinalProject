# Low-Cost Robot-Assisted Spinal Surgery Solution

The Low-Cost Robot-Assisted Spinal Surgery Solution package provides tools for calibrating cameras and controlling robotic movements based on 3D transformations. The system supports various cameras, including Intel RealSense and C-Arm devices, and utilizes AprilTags and custom blob detection algorithms for precise calibration and movement. 

## Nodes
**camera_calibration**: 
  1. Utilizes a blob detection algorithm to calibrate camera intrinsics of a C-arm. 
  2. Gets intrinics from an Intel Realsense Camera
  3. Returns the camera intrinsics for each camera type in a yaml file.

- To run, make sure realsense is running,  then `ros2 run robot camera_calibration_node `


**move_robot**: 
  1. Calculates the entry point, target point, trajectory, distance, and roll, pitch, yaw, to send movement commands to robot. 
  2. Entry point uses two images taken from the intel realsense, the desired entry point is selected in both images, and the 3D point relative to the world frame is retured.
  3. Target point uses the two x-ray images taken. The target point is selected in both images, and the 3D point relative to the world frame is returned.
  4. Trajectory: Target Point - Entry Point
  5. Will visualize all frames in RVIZ, and insert the entry point / trajectory into the rviz simulation as well. 
- To launch `ros2 launch robot node.launch.xml `

## Find Transforms
All relevant equations and transformations can be found in `Transforms.pdf`

## Acknowledgements
Thank you to Dr. Najib El Tecle for providing OR and X-ray access.




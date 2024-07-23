# Imports
from model_robot_functions import ModelRobot

# Step #1: Camera calibration -- when flag = false, assume that camer calibration is already complete

flag = False
image_directory = 'photos/'

# Initialize class, start livestream
model_robot = ModelRobot(image_directory, flag)
model_robot.startLivestream()

# Step #2: Image processing of top photo
# Step #2a: Detect apriltag
# Step #2b: Get top camera position information (T_topcam)

# Step #3: Image processing of side photo
# Step #3a: Detect apriltag
# Step #3b: Get side camera position information (T_sidecam)

# Step #4: Get side photo relative to top photo (T_sidecam-topcam)

# Step #5: Detect apriltag in livestream
# Step #5a: Get livestream camera position information (T_live-at)
# Step #5b: Save camera position 

# Step #6: Detect robot in livestream
# Step #6a: Get robot position

# Step #7: Consider livestream camera to be origin in world coordinates (T_cam)
# Step #7a: Get T_cam-topcam, T_cam-sidecam, T_cam-robot
# Step #7b: Annotate top and side photos with entry and target points
# Step #7c: Visualize (in world coordinates) the scene -- should include cam, topcam, sidecam, ray from topcam (tcR), ray from sidecam (scR)
# Step #7d: Determine trajectory
# step #7e: Get transformation from robot to trajectory (T_robot-traj)

# Step #8: Send trajectory to robot







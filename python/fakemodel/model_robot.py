# Imports
from model_robot_functions import ModelRobot
import threading


# Step #1: Camera calibration -- when flag = false, assume that camer calibration is already complete

flag = False
image_directory = 'photos/'

# Initialize class, start livestream
model_robot = ModelRobot(image_directory, flag)
stream_thread = threading.Thread(target=model_robot.start_livestream)
stream_thread.start()

while not model_robot.tags_detected:
    pass

# Step #2: Image processing of top photo
# Step #2a: Detect apriltag
id_top, top_tag = model_robot.detect_AprilTag_image('photos/fakeimg2.png')

# Step #2b: Get top camera position information (T_topcam)
T_topcam = model_robot.get_pose_from_image(id_top, top_tag)
print("Transformation of Top Camera:\n", T_topcam)

# Step #3: Image processing of side photo
# Step #3a: Detect apriltag
id_side, side_tag = model_robot.detect_AprilTag_image('photos/fakeimg3.png')

# Step #3b: Get side camera position information (T_sidecam)
T_sidecam = model_robot.get_pose_from_image(id_side, side_tag)
print("Transformation of Side Camera:\n", T_sidecam)

# Step #4: Get side photo relative to top photo (T_sidecam-topcam)

# Step #5: Detect apriltag in livestream
# Step #5a: Get livestream camera transformation
T_camera = model_robot.get_livestream_cam_pose()
print("Camera Transformation Matrix:\n", T_camera)


# Step #6: Detect robot in livestream
# Step #6a: Get robot position

# Step #7: Consider livestream camera to be origin in world coordinates (T_cam)
# Step #7a: Get T_cam-topcam, T_cam-sidecam, T_cam-robot
# Step #7b: Annotate top and side photos with entry and target points
# Step #7c: Visualize (in world coordinates) the scene -- should include cam, topcam, sidecam, ray from topcam (tcR), ray from sidecam (scR)
# Step #7d: Determine trajectory
# step #7e: Get transformation from robot to trajectory (T_robot-traj)

# Step #8: Send trajectory to robot

stream_thread.join()





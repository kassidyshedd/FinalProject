from image_processing import ImageProcessing


# Directory for calibration images
calib_image_dir = '/home/kashedd/finalproject/calibration_images'

# Directory for pose images
pose_image_dir = '/home/kashedd/finalproject/pose_images'

# Perform image calibration for camera intrinsics
image_processor = ImageProcessing(calib_image_dir)
image_processor.run_blob_detection()

# Get pose information from top view image

# Get pose information from side view image

# Get pose information from apriltag on fiducial

# Get pose information from april tag on robot


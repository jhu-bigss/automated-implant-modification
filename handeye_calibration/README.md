# Handeye Calibration
Calibrate RealSense cameras
KUKA LBR iiwa

## Requirements
- Python 3.8
- pyrealsense2
- RoboDK

## Robot Setup
See [iiwa_robodk](https://github.com/jhu-bigss/kuka_lbr_ros2/tree/main/iiwa_robodk) to set up a hand guiding application using RoboDK.

## Instructions
First, run "rs_image_capture_rgb.py" to perform camera calibration.
Second, run "rs_image_capture_handeye_calibration_rdk.py", which will read robot transformations from RoboDK and save to file. 
At the end, you can compute the hand-eye calibration result.

## Reference
- [Hand Eye Calibration](https://github.com/monroe-git/hand-eye)
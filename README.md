# Automatic Implant Localization
KUKA LBR iiwa - automatic implant localization with a RGBD sensor


## Pre-requisites
- Hardware
	- RGBD Sensor (Intel RealSense D400)
	- KUKA LBR iiwa
- Software
	- Windows 10/Ubuntu 18.04/20.04
		- [librealsense](https://github.com/IntelRealSense/librealsense)
		- [robodk](https://robodk.com/index)
		- KUKA Sunrise.Workbench 1.13

## Requirements
- Python 3.7 with [NumPy](http://www.numpy.org/), [SciPy](https://www.scipy.org/), [OpenCV](https://opencv.org/), [pyrealsense2](https://pypi.org/project/pyrealsense/), [robodk](https://pypi.org/project/robodk/), [pytransform3d](https://github.com/rock-learning/pytransform3d). These can be quickly installed/updated by running the following:
  ```shell
  pip install numpy scipy opencv-python pyrealsense2 robodk pytransform3d
  ```
## Descreption




## TO-DO
- [ ] Capture color and depth images with robot poses


## Reference
- [Hand Eye Calibration](https://github.com/monroe-git/hand-eye)
- [TSDF](https://github.com/andyzeng/tsdf-fusion-python)
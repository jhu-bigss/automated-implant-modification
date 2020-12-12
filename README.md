# Automatic Implant Localization
KUKA LBR iiwa - automatic implant localization with a RGBD sensor

## Pre-requisites
- Hardware
	- RGBD Sensor (Intel RealSense D400 Series or Primesense)
	- KUKA LBR iiwa
- Software
	- Windows 10/Ubuntu 18.04/20.04
		- [librealsense](https://github.com/IntelRealSense/librealsense) or [primesense](https://pypi.org/project/primesense/)
		- [robodk](https://robodk.com/index)
		- KUKA Sunrise.Workbench 1.13

## Requirements
- Python 3.7 with [NumPy](http://www.numpy.org/), [SciPy](https://www.scipy.org/), [OpenCV](https://opencv.org/), [pyrealsense2](https://pypi.org/project/pyrealsense/), [robodk](https://pypi.org/project/robodk/), [pytransform3d](https://github.com/rock-learning/pytransform3d). These can be quickly installed/updated by running the following:
```shell
pip install pyrealsense2 / primesense
```
```shell
pip install numpy scipy opencv-python robodk
```
For plotting hand-calibration result using matplotlib, [pytransform3d](https://pypi.org/project/pytransform3d/) is required:
```shell
pip install pytransform3d
```
To run volumetric TSDF Fusion of RGB-D images in Python, you need to install the following packages:
```shell
pip install scikit-image numba
```
If using GPU acceleration, it requires an NVIDIA GPU with [CUDA](https://developer.nvidia.com/cuda-downloads) and [PyCUDA](https://developer.nvidia.com/pycuda):
```shell
pip install pycuda
```
## Descreption



## TO-DO
- [ ] Port the toolpath generation code from izmar Github --> Improve toolpath generation

## Reference
- [Hand Eye Calibration](https://github.com/monroe-git/hand-eye)
- [TSDF](https://github.com/andyzeng/tsdf-fusion-python)
- [Python_OpenNI2](https://github.com/elmonkey/Python_OpenNI2/tree/master/samples) for Primesense
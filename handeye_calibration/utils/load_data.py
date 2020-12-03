import numpy as np
import glob
import cv2
from . import capture
from . import kinematics as kin


def read_robot_pose_from_dir(path):
    # ---------- Load the robot poses ---------- #
    pose_txt_list = sorted(glob.glob(path))
    robot_poses = []
    for file in pose_txt_list:
        pose = np.loadtxt(file, delimiter=',', usecols=range(4), unpack=True, dtype=float)
        robot_poses.append(pose)
    robot_poses = np.array(robot_poses)

    return robot_poses


def read_image_from_dir(path):
    # --------- Load the world poses ------------ #
    image_list = sorted(glob.glob(path))
    images = [cv2.imread(file) for file in image_list]
    c = capture.ChessBoard(0, 1920, 1080, 6, 4, 0.027)
    # ---------- Camera calibration ---------- #
    # calibrate(total_frame, time_btw_frame, live_view)
    # c.calibrate(images, 300, False)
    # ---------- Estimate the pose of calibration object w.r.t the camera ---------- #
    T_eye = c.estimatePose(images)
    return T_eye
    
import numpy as np
import glob
import cv2
from utils import chessboard

def read_robot_pose_from_dir(path):
    # ---------- Load the robot poses ---------- #
    pose_txt_list = sorted(glob.glob(path + "*.txt"))
    robot_poses = []
    for file in pose_txt_list:
        pose = np.loadtxt(file, delimiter=' ', usecols=range(4), dtype=float)
        robot_poses.append(pose)
    robot_poses = np.array(robot_poses)

    return robot_poses

def read_chessboard_image_from_dir(path, img_width=1280, img_height=720, chessboard_width=7, chessboard_height=6, chessboard_square_size=30, calibrate_camera=False):
    # --------- Load the world poses ------------ #
    image_list = sorted(glob.glob(path + "*.jpg"))
    images = [cv2.imread(file) for file in image_list]
    c = chessboard.ChessBoard(img_width, img_height, chessboard_width, chessboard_height, chessboard_square_size)

    # ---------- Camera calibration ---------- #
    if calibrate_camera:
        # calibrate(total_frame, time_btw_frame)
        c.calibrate(images, 300)
        return

    # ---------- Estimate the pose of calibration object w.r.t the camera ---------- #
    T_eye2world = c.estimate_pose(images)

    return T_eye2world

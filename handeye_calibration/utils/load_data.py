import os
import numpy as np
import glob
import cv2
from .chessboard import ChessBoard

data_foler = 'data/'

DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_CHESSBOARD_WIDTH = 7
DEFAULT_CHESSBOARD_HEIGHT = 6
DEFAULT_CHESSBOARD_SQUARE_SIZE = 30

def read_robot_pose_from_dir(path):
    # ---------- Load the robot poses ---------- #
    pose_txt_list = sorted(glob.glob(path + "*.txt"))
    robot_poses = []
    for file in pose_txt_list:
        pose = np.loadtxt(file, delimiter=' ', usecols=range(4), dtype=float)
        robot_poses.append(pose)
    robot_poses = np.array(robot_poses)

    return robot_poses


def read_image_from_dir(path, camera_calib_flag=False):
    # --------- Load the world poses ------------ #
    image_list = sorted(glob.glob(path + "*.png"))
    images = [cv2.imread(file) for file in image_list]
    c = ChessBoard(DEFAULT_WIDTH, DEFAULT_HEIGHT, DEFAULT_CHESSBOARD_WIDTH, DEFAULT_CHESSBOARD_HEIGHT, DEFAULT_CHESSBOARD_SQUARE_SIZE)

    # ---------- Camera calibration ---------- #
    if camera_calib_flag:
        # calibrate(total_frame, time_btw_frame)
        c.calibrate(images, 300)

    # ---------- Estimate the pose of calibration object w.r.t the camera ---------- #
    T_eye2world = c.estimate_pose(images)

    return T_eye2world

if __name__ == '__main__':

    # this_path = os.path.dirname(os.path.realpath(__file__)) + '/'
    data_path = os.path.join(os.getcwd() + '/', data_foler)
    robot_poses = read_robot_pose_from_dir(data_path)
    cam_estimate_poses = read_image_from_dir(data_path)
    print("-------------robot---------------")
    print("load", len(robot_poses), "robot poses")
    print("------------camera--------------")
    print("load", len(cam_estimate_poses), "images")
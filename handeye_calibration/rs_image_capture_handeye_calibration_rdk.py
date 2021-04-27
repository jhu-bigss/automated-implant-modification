"""
RoboDK capture images with RealSense camera based on PyQt
"""

import os, sys, shutil, glob
import numpy as np

from PyQt5 import Qt, QtCore

from robolink import *
from robodk import *

from utils.camerathread import CameraThread
from utils import calibrate
from utils import compute_error
from utils import chessboard

data_foler = 'data/'

methods = ['TSA', 'PAR', 'HOR', 'AND', 'DAN', 'AXC1', 'AXC2', 'HORN', 'OURS', 'LI', 'SHA', 'TZC1', 'TZC2']
exclude_method = [10, 11, 12, 13]
observ_rank = False # Rank of the matrix
observ_pose = False # Use redundant stations
verification = False # Use a verification in other poses
validate_folder = 'data_validate/'
show_plot = True

class MainWidget(Qt.QWidget):
    rdk = Robolink()
    image_captured = QtCore.pyqtSignal(str)
    window_closed = QtCore.pyqtSignal()

    def __init__(self, name=None, parent=None, show=True):
        super(MainWidget, self).__init__()
        self.setWindowTitle(name)

        # create a label to display camera image
        self.cv_label = Qt.QLabel()
        cv_thread = CameraThread(self, 1920, 1080, 30)
        cv_thread.change_pixmap.connect(self.set_image)
        cv_thread.start()
        self.data_directory = self.create_data_directory(os.path.dirname(os.path.realpath(__file__)))

        self.open_dir_button = Qt.QPushButton("Open Folder")
        self.capture_button = Qt.QPushButton("Capture")
        self.calibrate_button = Qt.QPushButton("Calibrate")

        self.open_dir_button.clicked.connect(self.open_data_directory)
        self.capture_button.clicked.connect(self.capture_event)
        self.calibrate_button.clicked.connect(self.calibrate_handeye)

        vlayout = Qt.QVBoxLayout()
        vlayout.addWidget(self.cv_label)
        hlayout_1 = Qt.QHBoxLayout()
        hlayout_1.addWidget(self.open_dir_button)
        hlayout_1.addWidget(self.capture_button)
        hlayout_1.addWidget(self.calibrate_button)
        vlayout.addLayout(hlayout_1)
        vlayout.setContentsMargins(0,0,0,0)
        self.setLayout(vlayout)

        # RoboDK stuff
        self.robot = self.rdk.Item('', ITEM_TYPE_ROBOT)
        self.pose_counter = 0

        if show:
            self.show()


    def closeEvent(self, event):
        reply = Qt.QMessageBox.question(self, 'Window Close', 'Are you sure you want to close this window?',
                Qt.QMessageBox.Yes | Qt.QMessageBox.No, Qt.QMessageBox.No)

        if reply == Qt.QMessageBox.Yes:
            # signal camera thread to stop camera
            self.window_closed.emit()
            event.accept()
            print('Window closed')
        else:
            event.ignore()

    @Qt.pyqtSlot(Qt.QImage)
    def set_image(self, image):
        self.cv_label.setPixmap(Qt.QPixmap.fromImage(image))

    # create a directory to save captured images 
    def create_data_directory(self, dir):
        dir = os.path.join(dir, data_foler)
        # import datetime
        # now = datetime.datetime.now()
        # dir = os.path.join(dir, now.strftime("%Y-%m-%d-%H%M%S"))

        try:
            if not(os.path.isdir(dir)):
                os.makedirs(dir)
        except OSError as e:
            print("Can't make the directory: %s" % dir)
            raise
        return dir
    
    # open an existing directory for calibration
    def open_data_directory(self):
        dir_open = Qt.QFileDialog.getExistingDirectory(self, 'Open Folder', os.path.dirname(os.path.realpath(__file__)))
        self.data_directory = self.create_data_directory(dir_open)

    # call the opencv thread to save image to the given directory
    def capture_event(self):
        self.image_captured.emit(self.data_directory)

        # save the robot's current pose
        robot_pose = self.robot.Pose()
        f_name = 'frame-%06d.pose.txt'%self.pose_counter
        robot_pose.tr().SaveMat(f_name, separator=' ')
        shutil.move(os.path.join(os.getcwd(), f_name), os.path.join(self.data_directory, f_name))
        print('robot pose: ' + str(self.pose_counter))
        self.pose_counter += 1

    @staticmethod
    def validate(A, B, X_est):
        """validate.

        :param A: Numpy array, (total_poses, 4, 4)
        :param B: Numpy array, (total_poses, 4, 4)
        :param X_est: Numpy array, (total_methods, 4, 4)
        :return pos_e, ori_e: Numpy array (1, total_method)
        """

        pos_e = []
        ori_e = []
        for X_est_ in X_est:
            pos_e_, ori_e_ = compute_error.woGroundTruth(A, B, X_est_)
            pos_e.append(pos_e_)
            ori_e.append(ori_e_)
        return pos_e, ori_e
    
    @staticmethod
    def read_robot_pose_from_dir(path):
        # ---------- Load the robot poses ---------- #
        pose_txt_list = sorted(glob.glob(path + "*.txt"))
        robot_poses = []
        for file in pose_txt_list:
            pose = np.loadtxt(file, delimiter=' ', usecols=range(4), dtype=float)
            robot_poses.append(pose)
        robot_poses = np.array(robot_poses)

        return robot_poses

    def calibrate_handeye(self):

        if not exclude_method:
            total_method = 13
            ex_method = []
        else:
            total_method = 13 - len(exclude_method)
            ex_method = list(map(int, exclude_method))

        for idx, i in enumerate(ex_method):
            del methods[(i-1)-idx]

        lm_tol = 1e-4
        R_tol = 1e-7

        # Load data
        T_robot = self.read_robot_pose_from_dir(self.data_directory)
        T_eye = chessboard.read_chessboard_image_from_dir(self.data_directory, 1920, 1080, 7, 6, 30)
        num_of_poses = len(T_eye)
        if num_of_poses != len(T_robot):
            print("WARNING: load robot poses not equal to the number of images.")

        # To validate the new pose using the estimated result
        # T_robot_vali = load_data.read_robot_pose_from_dir(validate_folder)
        # T_eye_vali = load_data.read_chessboard_image_from_dir(validate_folder)

        min_num_pose = 9
        max_num_pose = len(T_eye)
        
        if observ_pose:
            print('Effect of the number of poses')
            pos_e = np.empty((1, total_method))
            ori_e = np.empty((1, total_method))
            for n_pose in range(min_num_pose, max_num_pose + 1):
                print(n_pose, 'poses')
                A = T_robot[:n_pose, :]
                B = T_eye[:n_pose, :]
                X_axxb = calibrate.axxb_method(A, B, lm_tol, ex_method)
                X_axyb, Y_axyb = calibrate.axyb_method(A, B, lm_tol, ex_method)
                X_est = np.concatenate((X_axxb, Y_axyb), axis=0)
                if verification:
                    pos_e_, ori_e_ = compute_error.woGroundTruth(T_robot_vali, T_eye_vali, X_est, R_tol)
                    pos_e = np.concatenate((pos_e, np.expand_dims(pos_e_, axis=0)), axis=0)
                    ori_e = np.concatenate((ori_e, np.expand_dims(ori_e_, axis=0)), axis=0)
                else:
                    # TODO
                    # 1. T_robot, T_eye : total or calib or vali?
                    # 2. Total error = position error + orientation error? no weight? 
                    pos_e, ori_e = compute_error.woGroundTruth(T_robot, T_eye, X_est, R_tol)
                    err = pos_e + ori_e
                    min_idx = np.argmin(err)
                    print('X(at the minumum error): ')
                    print(X_est[min_idx])
            if verification:
                compute_error.graphPlot(pos_e[1:], ori_e[1:], ex_method, 1)

        elif observ_rank:
            from utils import axxb
            M_rank, N_rank = axxb.Tsai_rank(T_robot, T_eye)
            print("M rank :", M_rank)
            print("N rank :", N_rank)
            
        else:
            X_axxb = calibrate.axxb_method(T_robot, T_eye, lm_tol, ex_method)
            X_axyb, Y_axyb = calibrate.axyb_method(T_robot, T_eye, lm_tol, ex_method)
            X_est = np.concatenate((X_axxb, Y_axyb), axis=0)
            pos_e, ori_e = compute_error.woGroundTruth(T_robot, T_eye, X_est, R_tol)
            err = pos_e + ori_e
            min_idx = np.argmin(err)
            np.set_printoptions(suppress=True)
            print('X(at the minumum error):')
            print(X_est[min_idx])
            
            if show_plot:
                self.plot_handeye_result(X_est[min_idx])

    def plot_handeye_result(self, T_hand2eye):
        import matplotlib.pyplot as plt
        from pytransform3d.plot_utils import make_3d_axis
        from pytransform3d.transformations import random_transform
        from pytransform3d.transform_manager import TransformManager
        
        tm = TransformManager()
        # add_transform(child_frame, parent_frame, H):
        tm.add_transform("cam", "ee", T_hand2eye)
        plt.figure(figsize=(5, 5))
        ax = make_3d_axis(100)
        ax = tm.plot_frames_in("ee", ax=ax, s=50) # scaling 50
        ax.view_init()
        plt.show()

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window_title="OpenCV Hand-eye Calibration"
    window = MainWidget(window_title)
    sys.exit(app.exec_())
"""
RoboDK capture images with RealSense camera based on PyQt
"""

import os, sys, shutil
import numpy as np
import cv2

from PyQt5 import Qt, QtCore

from robolink import *
from robodk import *

from utils.camerathread import CameraThread
from utils import fusion

data_foler = 'data/'

class MainWidget(Qt.QWidget):
    rdk = Robolink()
    view_mode_changed = QtCore.pyqtSignal(bool)
    image_captured = QtCore.pyqtSignal(str)
    window_closed = QtCore.pyqtSignal()

    def __init__(self, name=None, parent=None, show=True):
        super(MainWidget, self).__init__()
        self.setWindowTitle(name)

        # create a label to display camera image
        self.cv_label = Qt.QLabel()
        cv_thread = CameraThread(self)
        cv_thread.change_pixmap.connect(self.set_image)
        cv_thread.start()

        # create default save folder
        self.save_directory = self.create_data_directory(os.path.dirname(os.path.realpath(__file__)))

        self.open_dir_button = Qt.QPushButton("Save Folder")
        self.toggle_view_mode_button = Qt.QPushButton("View Mode")
        self.toggle_view_mode_button.setCheckable(True)
        self.toggle_view_mode_button.setStyleSheet("background-color : lightgrey")
        self.capture_button = Qt.QPushButton("Capture")
        self.calibrate_button = Qt.QPushButton("Run TSDF")

        self.open_dir_button.clicked.connect(self.open_data_directory)
        self.toggle_view_mode_button.clicked.connect(self.change_view_mode)
        self.capture_button.clicked.connect(self.capture_event)
        self.calibrate_button.clicked.connect(self.run_tsdf)

        vlayout = Qt.QVBoxLayout()
        vlayout.addWidget(self.cv_label)
        hlayout_1 = Qt.QHBoxLayout()
        hlayout_1.addWidget(self.open_dir_button)
        hlayout_1.addWidget(self.toggle_view_mode_button)
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

    # create a directory to save captured images and robot poses
    def create_data_directory(self, dir):
        dir = os.path.join(dir, data_foler)
        try:
            if not(os.path.isdir(dir)):
                os.makedirs(dir)
        except OSError as e:
            print("Can't make the directory: %s" % dir)
            raise
        return dir 
    
    # open a new directory for saving data
    def open_data_directory(self):
        dir_open = Qt.QFileDialog.getExistingDirectory(self, 'Select Save Folder', os.path.dirname(os.path.realpath(__file__)))
        self.save_directory = self.create_data_directory(dir_open)

    def change_view_mode(self):
        if self.toggle_view_mode_button.isChecked():
            self.toggle_view_mode_button.setStyleSheet("background-color : lightblue")
            self.view_mode_changed.emit(True)
        else:
            self.toggle_view_mode_button.setStyleSheet("background-color : lightgrey")
            self.view_mode_changed.emit(False)

    # call the opencv thread to save image to the given directory
    def capture_event(self):
        self.image_captured.emit(self.save_directory)

        # save the robot's current pose
        robot_pose = self.robot.Pose()
        f_name = 'frame-%06d.pose.txt'%self.pose_counter
        robot_pose.tr().SaveMat(f_name, separator=' ')
        shutil.move(os.path.join(os.getcwd(), f_name), os.path.join(self.save_directory, f_name))
        print('robot pose: ' + str(self.pose_counter))
        self.pose_counter += 1

    def run_tsdf(self):
        vol_center = np.array([500, 0, 450]) 
        vol_width = 800
        vol_height = 80
        vol_bnds = np.zeros((3,2))
        vol_bnds[:,0] = vol_center - np.array([vol_width/2, vol_width/2, vol_height/2])
        vol_bnds[:,1] = vol_center + np.array([vol_width/2, vol_width/2, vol_height/2])
        vol_size = 1 # mm

        # ======================================================================================================== #
        # (Optional) This is an example of how to compute the 3D bounds
        # in world coordinates of the convex hull of all camera view
        # frustums in the dataset
        # ======================================================================================================== #
        print("Estimating voxel volume bounds...")
        n_imgs = 11
        cam_intr = np.loadtxt("data/camera-intrinsics.txt", delimiter=' ')
        for i in range(n_imgs):
            # Read depth image and camera pose
            depth_im = cv2.imread("data/frame-%06d.depth.png"%(i),-1).astype(float) # depth is saved in 16-bit PNG in millimeters
            depth_im[depth_im == 65535] = 0  # invalid depth is set to 65535
            cam_pose = np.loadtxt("data/frame-%06d.pose.txt"%(i))  # 4x4 rigid transformation matrix

            # Compute camera view frustum and extend convex hull
            # view_frust_pts = fusion.get_view_frustum(depth_im, cam_intr, cam_pose)
            # vol_bnds[:,0] = np.minimum(vol_bnds[:,0], np.amin(view_frust_pts, axis=1))
            # vol_bnds[:,1] = np.maximum(vol_bnds[:,1], np.amax(view_frust_pts, axis=1))
        # ======================================================================================================== #

        # ======================================================================================================== #
        # Integrate
        # ======================================================================================================== #
        # Initialize voxel volume
        print("Initializing voxel volume...")
        tsdf_vol = fusion.TSDFVolume(vol_bnds, voxel_size=vol_size)

        # Loop through RGB-D images and fuse them together
        t0_elapse = time.time()
        for i in range(n_imgs):
            print("Fusing frame %d/%d"%(i+1, n_imgs))

            # Read RGB-D image and camera pose
            color_image = cv2.cvtColor(cv2.imread("data/frame-%06d.color.jpg"%(i)), cv2.COLOR_BGR2RGB)
            depth_im = cv2.imread("data/frame-%06d.depth.png"%(i),-1).astype(float)
            depth_im[depth_im == 65535] = 0
            cam_pose = np.loadtxt("data/frame-%06d.pose.txt"%(i))

            # Integrate observation into voxel volume (assume color aligned with depth)
            tsdf_vol.integrate(color_image, depth_im, cam_intr, cam_pose, obs_weight=1.)

        fps = n_imgs / (time.time() - t0_elapse)
        print("Average FPS: {:.2f}".format(fps))

        # Get mesh from voxel volume and save to disk (can be viewed with Meshlab)
        print("Saving mesh to mesh.ply...")
        verts, faces, norms, colors = tsdf_vol.get_mesh()
        fusion.meshwrite("mesh.ply", verts, faces, norms, colors)

        # Get point cloud from voxel volume and save to disk (can be viewed with Meshlab)
        print("Saving point cloud to pc.ply...")
        point_cloud = tsdf_vol.get_point_cloud()
        fusion.pcwrite("pc.ply", point_cloud)

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window_title="OpenCV Hand-eye Calibration"
    window = MainWidget(window_title)
    sys.exit(app.exec_())
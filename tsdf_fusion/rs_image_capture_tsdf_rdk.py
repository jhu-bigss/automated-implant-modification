"""
RoboDK capture images with RealSense camera based on PyQt
"""

import os, sys, shutil, time, glob
import numpy as np
import cv2

from PyQt5 import Qt, QtCore

from utils.camerathread import CameraRealsense
from utils.rdkthread import RoboDK
from utils import fusion

data_foler = 'data/'

class MainWidget(Qt.QWidget):

    view_mode_changed = QtCore.pyqtSignal(bool)
    set_save_dir = QtCore.pyqtSignal(str)
    image_capture = QtCore.pyqtSignal()
    pose_capture = QtCore.pyqtSignal()
    close_window = QtCore.pyqtSignal()

    def __init__(self, name=None, parent=None, show=True):
        super(MainWidget, self).__init__()
        self.setWindowTitle(name)

        # create default save folder
        self.data_directory = self.create_data_directory(os.path.dirname(os.path.realpath(__file__)))

        # create a label to display camera image
        self.cv_label = Qt.QLabel()
        # Choose RealSense or Primesense camera; if use SR300, it requires 640x480
        cv_thread = CameraRealsense(self, 640, 480)
        # cv_thread = CameraPrimesense(self)
        cv_thread.change_pixmap.connect(self.set_Qimage)
        cv_thread.start()

        # Robot Communication through RoboDK
        self.rdk = RoboDK(self)
        # Run tsdf immediately after capturing all the data in automatic mode
        self.rdk.automatic_capture_complete.connect(self.run_tsdf)

        self.open_dir_button = Qt.QPushButton("Save Folder")
        self.toggle_view_mode_button = Qt.QPushButton("View Mode")
        self.toggle_view_mode_button.setCheckable(True)
        self.toggle_view_mode_button.setStyleSheet("background-color : lightgrey")
        self.toggle_tsdf_mode_button = Qt.QPushButton("TSDF Mode")
        self.toggle_tsdf_mode_button.setCheckable(True)
        self.toggle_tsdf_mode_button.setStyleSheet("background-color : lightgrey")
        self.capture_button = Qt.QPushButton("Capture")

        # Parameters inputs for TSDF
        self.vol_width_dspinbox = Qt.QDoubleSpinBox()
        self.vol_width_dspinbox.setMaximum(1000)
        self.vol_width_dspinbox.setValue(300)
        self.vol_height_dspinbox = Qt.QDoubleSpinBox()
        self.vol_height_dspinbox.setMaximum(200)
        self.vol_height_dspinbox.setValue(50)
        self.vol_marching_cube_size = Qt.QDoubleSpinBox()
        self.vol_marching_cube_size.setSingleStep(0.1)
        self.vol_marching_cube_size.setValue(1)
        self.run_tsdf_button = Qt.QPushButton("Run TSDF")

        self.open_dir_button.clicked.connect(self.open_data_directory)
        self.toggle_view_mode_button.clicked.connect(self.change_view_mode)
        self.toggle_tsdf_mode_button.clicked.connect(self.change_tsdf_mode)
        self.capture_button.clicked.connect(self.capture_event)
        self.run_tsdf_button.clicked.connect(self.run_tsdf)

        # GUI layouts
        vlayout = Qt.QVBoxLayout()
        vlayout.addWidget(self.cv_label)
        hlayout_0 = Qt.QHBoxLayout()
        hlayout_0.addWidget(self.open_dir_button)
        hlayout_0.addWidget(self.toggle_view_mode_button)
        hlayout_0.addWidget(self.toggle_tsdf_mode_button)
        hlayout_0.addWidget(self.capture_button)
        hlayout_0.addWidget(self.run_tsdf_button)
        vlayout.addLayout(hlayout_0)
        hlayout_1 = Qt.QHBoxLayout()
        hlayout_1.addWidget(Qt.QLabel("TSDF Parameters: "))
        hlayout_1.addWidget(Qt.QLabel("Width"))
        hlayout_1.addWidget(self.vol_width_dspinbox)
        hlayout_1.addWidget(Qt.QLabel("Height"))
        hlayout_1.addWidget(self.vol_height_dspinbox)
        hlayout_1.addWidget(Qt.QLabel("Resolution"))
        hlayout_1.addWidget(self.vol_marching_cube_size)
        vlayout.addLayout(hlayout_1)
        vlayout.setContentsMargins(0,0,0,0)
        self.setLayout(vlayout)

        if show:
            self.show()

    def closeEvent(self, event):
        reply = Qt.QMessageBox.question(self, 'Window Close', 'Are you sure you want to close this window?',
                Qt.QMessageBox.Yes | Qt.QMessageBox.No, Qt.QMessageBox.No)

        if reply == Qt.QMessageBox.Yes:
            # signal camera thread to stop camera and robodk
            self.close_window.emit()
            event.accept()
        else:
            event.ignore()

    @Qt.pyqtSlot(Qt.QImage)
    def set_Qimage(self, image):
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
        self.data_directory = self.create_data_directory(dir_open)
        self.set_save_dir.emit(self.data_directory)

    def change_view_mode(self):
        if self.toggle_view_mode_button.isChecked():
            self.toggle_view_mode_button.setText("Depth View")
            self.toggle_view_mode_button.setStyleSheet("background-color : lightblue")
            self.view_mode_changed.emit(True)
        else:
            self.toggle_view_mode_button.setText("Color View")
            self.toggle_view_mode_button.setStyleSheet("background-color : lightgrey")
            self.view_mode_changed.emit(False)
    
    def change_tsdf_mode(self):
        if self.toggle_tsdf_mode_button.isChecked():
            self.toggle_tsdf_mode_button.setText("Automatic")
            self.toggle_tsdf_mode_button.setStyleSheet("background-color : lightblue")
            self.capture_button.setText("Start")
            self.run_tsdf_button.setEnabled(False)
        else:
            self.toggle_tsdf_mode_button.setText("Manual")
            self.toggle_tsdf_mode_button.setStyleSheet("background-color : lightgrey")
            self.capture_button.setText("Capture")
            self.run_tsdf_button.setEnabled(True)

    def capture_event(self):
        if self.toggle_tsdf_mode_button.isChecked():
            # Automatic mode
            self.rdk.start()
        else:
            # Manual mode
            self.image_capture.emit()
            self.pose_capture.emit()

    def run_tsdf(self):
        vol_bnds = np.zeros((3,2))
        vol_bnds[:,0] = np.array(self.rdk.get_ref_frame()) - np.array([self.vol_width_dspinbox.value()/2, self.vol_width_dspinbox.value()/2, 0])
        vol_bnds[:,1] = np.array(self.rdk.get_ref_frame()) + np.array([self.vol_width_dspinbox.value()/2, self.vol_width_dspinbox.value()/2, self.vol_height_dspinbox.value()])

        # ======================================================================================================== #
        # Integrate
        # ======================================================================================================== #
        # Initialize voxel volume
        print("Initializing voxel volume...")
        tsdf_vol = fusion.TSDFVolume(vol_bnds, voxel_size=self.vol_marching_cube_size.value())

        # Detect how many frames in data directory
        n_imgs = len(glob.glob1(self.data_directory, "*.jpg"))

        # Load camera intrinsics
        calib_file = os.path.join(self.data_directory, '../camera_params.yaml')
        calib_file = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
        cam_intr = calib_file.getNode("intrinsic").mat()

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

        print(type(tsdf_vol))
        print(tsdf_vol)

        # # Get mesh from voxel volume and save to disk (can be viewed with Meshlab)
        # print("Saving mesh to mesh.ply...")
        # verts, faces, norms, colors = tsdf_vol.get_mesh()
        # fusion.meshwrite("mesh.ply", verts, faces, norms, colors)

        # # Get point cloud from voxel volume and save to disk (can be viewed with Meshlab)
        # print("Saving point cloud to pc.ply...")
        # point_cloud = tsdf_vol.get_point_cloud()
        # fusion.pcwrite("pc.ply", point_cloud)

        print("=====>Done<=====")

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window_title="OpenCV Hand-eye Calibration"
    window = MainWidget(window_title)
    sys.exit(app.exec_())
"""
RoboDK capture images with RealSense camera based on PyQt
"""

import os, sys, shutil, time, glob
import numpy as np
import cv2

from PyQt5 import Qt, QtCore

from robolink import *
from robodk import robodk

from utils.camerathread import CameraThread
from utils import fusion

data_foler = 'data/'

reference_frame_wrt_robot_base = [772, -8, 410]
auto_scan_camera_to_object_distance = 400
auto_scan_tilt_angles = [80, 60]
auto_scan_planer_swing_angles = [[-45, 0, 45],
                                 [-50, -30, -15, 0, 15, 30, 50]]

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
        self.data_directory = self.create_data_directory(os.path.dirname(os.path.realpath(__file__)))

        self.open_dir_button = Qt.QPushButton("Save Folder")
        self.toggle_view_mode_button = Qt.QPushButton("View Mode")
        self.toggle_view_mode_button.setCheckable(True)
        self.toggle_view_mode_button.setStyleSheet("background-color : lightgrey")
        self.toggle_tsdf_mode_button = Qt.QPushButton("TSDF Mode")
        self.toggle_tsdf_mode_button.setCheckable(True)
        self.toggle_tsdf_mode_button.setStyleSheet("background-color : lightgrey")
        self.capture_button = Qt.QPushButton("Capture")

        # Parameters inputs for TSDF
        self.vol_bottom_center_wrt_robot_base = np.array(reference_frame_wrt_robot_base)
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

        # RoboDK stuff
        self.robot = self.rdk.Item('', ITEM_TYPE_ROBOT)
        self.ref_frame = self.rdk.Item('Reference', ITEM_TYPE_FRAME)
        if not self.ref_frame.Valid():
            self.ref_frame = self.rdk.AddFrame('Reference')
            self.ref_frame.setPose(robodk.transl(*reference_frame_wrt_robot_base))
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
        self.data_directory = self.create_data_directory(dir_open)

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
            targets = self.generate_automatic_poses()
            if self.connect_robot():
                for i, target in enumerate(targets):
                    self.robot.MoveJ(target)
                    self.robot.WaitMove(10) # in seconds
                    self.image_captured.emit(self.data_directory)
                    self.save_robot_pose(i)
                self.robot.Disconnect()
            
            self.run_tsdf()

        else:
            # Manual mode
            self.image_captured.emit(self.data_directory) # call the opencv thread to save image to the given directory
            self.save_robot_pose(self.save_robot_pose)
            self.pose_counter += 1

    def generate_automatic_poses(self):
        target_list = []
        r = auto_scan_camera_to_object_distance
        pose_counter = 0

        for i, tilt_angle in enumerate(auto_scan_tilt_angles):
            for swing_angle in auto_scan_planer_swing_angles[i]:
                # Pose position and orientation
                x = - r * robodk.cos(tilt_angle*robodk.pi/180) * robodk.cos(swing_angle*robodk.pi/180)
                y = r * robodk.cos(tilt_angle*robodk.pi/180) * robodk.sin(swing_angle*robodk.pi/180)
                z = r * robodk.sin(tilt_angle*robodk.pi/180)
                pose_offset_from_ref = robodk.Offset(self.ref_frame, x, y, z)
                pose_offset_from_ref = pose_offset_from_ref * robodk.rotz(-(swing_angle+90)*robodk.pi/180) * robodk.rotx(-(tilt_angle+90)*robodk.pi/180)
                self.robot.setPose(pose_offset_from_ref)
                # Add new target
                target = self.rdk.AddTarget(str(pose_counter))
                target.setAsJointTarget()
                target_list.append(target)
                pose_counter += 1

        return target_list

    def connect_robot(self):
        # Update connection parameters if required:
        # robot.setConnectionParams('192.168.2.35',30000,'/', 'anonymous','')
        
        # Connect to the robot using default IP
        success = self.robot.Connect() # Try to connect once
        #success robot.ConnectSafe() # Try to connect multiple times
        status, status_msg = self.robot.ConnectedState()
        if status != ROBOTCOM_READY:
            # Stop if the connection did not succeed
            print(status_msg)
            raise Exception("Failed to connect: " + status_msg)
        
        # This will set to run the API programs on the robot and the simulator (online programming)
        self.rdk.setRunMode(RUNMODE_RUN_ROBOT)
        # Note: This is set automatically when we Connect() to the robot through the API

        self.robot.setSpeedJoints(100)

        return success

    def save_robot_pose(self, pose_counter):
        # save the robot's current pose
        robot_pose = self.robot.Pose()
        f_name = 'frame-%06d.pose.txt'%pose_counter
        robot_pose.tr().SaveMat(f_name, separator=' ')
        shutil.move(os.path.join(os.getcwd(), f_name), os.path.join(self.data_directory, f_name))
        print('robot pose: ' + str(pose_counter))

    def run_tsdf(self):
        vol_bnds = np.zeros((3,2))
        vol_bnds[:,0] = self.vol_bottom_center_wrt_robot_base - np.array([self.vol_width_dspinbox.value()/2, self.vol_width_dspinbox.value()/2, 0])
        vol_bnds[:,1] = self.vol_bottom_center_wrt_robot_base + np.array([self.vol_width_dspinbox.value()/2, self.vol_width_dspinbox.value()/2, self.vol_height_dspinbox.value()])

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

        # Get mesh from voxel volume and save to disk (can be viewed with Meshlab)
        print("Saving mesh to mesh.ply...")
        verts, faces, norms, colors = tsdf_vol.get_mesh()
        fusion.meshwrite("mesh.ply", verts, faces, norms, colors)

        # Get point cloud from voxel volume and save to disk (can be viewed with Meshlab)
        print("Saving point cloud to pc.ply...")
        point_cloud = tsdf_vol.get_point_cloud()
        fusion.pcwrite("pc.ply", point_cloud)

        print("=====>Done<=====")

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window_title="OpenCV Hand-eye Calibration"
    window = MainWidget(window_title)
    sys.exit(app.exec_())
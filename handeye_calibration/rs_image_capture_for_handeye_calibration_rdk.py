"""
RoboDK capture images with RealSense camera based on PyQt
"""

import os, sys, datetime

from PyQt5 import Qt, QtCore

from robolink import *
from robodk import *

from utils.camerathread import CameraThread

data_foler = 'data/'

class MainWidget(Qt.QWidget):
    rdk = Robolink()
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
        self.save_directory = self.create_image_directory(os.path.dirname(os.path.realpath(__file__)))

        self.open_dir_button = Qt.QPushButton("Open Folder")
        self.capture_button = Qt.QPushButton("Capture")
        self.calibrate_button = Qt.QPushButton("Calibrate")

        self.open_dir_button.clicked.connect(self.open_image_directory)
        self.capture_button.clicked.connect(self.capture_event)
        self.calibrate_button.clicked.connect(self.calibrate_event)

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
    def create_image_directory(self, dir):
        dir = os.path.join(dir, data_foler)
        now = datetime.datetime.now()
        dir = os.path.join(dir, now.strftime("%Y-%m-%d-%H%M%S"))
        try:
            if not(os.path.isdir(dir)):
                os.makedirs(dir)
        except OSError as e:
            print("Can't make the directory: %s" % dir)
            raise
        return dir
    
    # open an existing directory for calibration
    def open_image_directory(self):
        dir_open = Qt.QFileDialog.getExistingDirectory(self, 'Open Folder', os.path.dirname(os.path.realpath(__file__)))
        self.save_directory = self.create_image_directory(dir_open)

    # call the opencv thread to save image to the given directory
    def capture_event(self):
        self.image_captured.emit(self.save_directory)

        # save the robot's current pose
        robot_pose = self.robot.Pose()
        f_name = 'frame-%06d.pose.txt'%self.pose_counter
        robot_pose.tr().SaveMat(f_name, separator=' ')
        shutil.move(os.path.join(os.getcwd(), f_name), self.save_directory)
        print('robot pose: ' + str(self.pose_counter))
        self.pose_counter += 1

    def calibrate_event(self):
        pass

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window_title="OpenCV Hand-eye Calibration"
    window = MainWidget(window_title)
    sys.exit(app.exec_())
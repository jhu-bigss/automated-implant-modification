import os, shutil

from PyQt5 import Qt, QtCore

from robolink import *
from robodk import robodk

# Scanning parameters:
auto_scan_camera_to_object_distance = 300
reference_frame_wrt_robot_base = [772, -8, 420]
auto_scan_tilt_angles = [80]
auto_scan_planer_swing_angles = [[-30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30]]

class RoboDK(QtCore.QThread):

    rdk = Robolink()
    automatic_capture_complete = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__()

        self.robot = self.rdk.Item('', ITEM_TYPE_ROBOT)

        self.ref_frame = self.rdk.Item('Reference', ITEM_TYPE_FRAME)
        if not self.ref_frame.Valid():
            self.ref_frame = self.rdk.AddFrame('Reference')
            self.ref_frame.setPose(robodk.transl(*reference_frame_wrt_robot_base))

        parent.set_save_dir.connect(self.set_save_dir)
        parent.pose_capture.connect(self.save_robot_pose)
        parent.window_closed.connect(self.quit)

        self.save_dir = parent.data_directory
        self.image_capture = parent.image_capture

        self.pose_counter = 0

    def run(self):
        targets = self.generate_automatic_poses()
        if self.connect_robot():
            for target in targets:
                self.robot.MoveJ(target)
                self.robot.WaitMove(10) # in seconds
                self.image_capture.emit()
                self.save_robot_pose()
            self.robot.Disconnect()
            self.automatic_capture_complete.emit()

    def get_ref_frame(self):
        # return the translation part of the reference frame pose
        xyz = self.ref_frame.Pose().list2()[-1]
        # pop out the last element since list2() gives a homogeneous point
        xyz.pop()
        return xyz

    def generate_automatic_poses(self):
        target_list = []
        target_cnt = 1
        r = auto_scan_camera_to_object_distance

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
                target = self.rdk.AddTarget(str(target_cnt))
                target.setAsJointTarget()
                target_list.append(target)
                target_cnt += 1

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

    @Qt.pyqtSlot(str)
    def set_save_dir(self, dir):
        self.save_dir = dir

    @Qt.pyqtSlot()
    def save_robot_pose(self):
        # save the robot's current pose
        robot_pose = self.robot.Pose()
        f_name = 'frame-%06d.pose.txt'%self.pose_counter
        robot_pose.tr().SaveMat(f_name, separator=' ')
        shutil.move(os.path.join(os.getcwd(), f_name), os.path.join(self.save_dir, f_name))
        print('robot pose: ' + str(self.pose_counter))
        self.pose_counter += 1
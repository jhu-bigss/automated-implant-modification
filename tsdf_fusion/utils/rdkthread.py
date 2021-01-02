import os, shutil

from PyQt5 import Qt, QtCore

from robolink import *
from robodk import robodk

from .rdkpathplanner import PathPlanner

class RoboDK(QtCore.QThread):

    rdk = Robolink()
    automatic_capture_complete = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__()

        # check if RoboDK application is running
        if not self.rdk.Connect():
            print("Warning: RoboDK is not running.")
            return

        self.robot = self.rdk.Item('', ITEM_TYPE_ROBOT)

        # Check if reference frame already existed, if yes, use it; if not, add new one
        ref_frame_query = self.rdk.Item('Reference', ITEM_TYPE_FRAME)
        if ref_frame_query.Valid():
            self.ref_frame = ref_frame_query
        else:
            self.ref_frame = self.rdk.AddFrame('Reference')

        parent.set_save_dir.connect(self.set_save_dir)
        parent.pose_capture.connect(self.save_robot_pose)
        parent.update_ref_frame.connect(self.update_ref_frame)
        parent.update_scan_circle.connect(self.update_scan_circle)
        parent.close_window.connect(self.quit)

        self.save_dir = parent.data_directory # data_dir path from parent
        self.image_capture = parent.image_capture # grab image_capture signal from parent

        self.pose_counter = 0

    def run(self):
        targets = self.generate_circular_poses()
        if self.connect_robot():
            for target in targets:
                self.robot.MoveJ(target)
                self.robot.WaitMove(10) # in seconds
                self.image_capture.emit()
                self.save_robot_pose()
            self.robot.Disconnect()
            self.automatic_capture_complete.emit()

    @Qt.pyqtSlot(object)
    def update_ref_frame(self, value):
        # input value is a list of [x, y, z]
        self.ref_frame.setPose(robodk.transl(*value))

    @Qt.pyqtSlot(object)
    def update_scan_circle(self, value):
        # input value is a list of [h, r, num_of_poses]
        self.circle_height = value[0]
        self.circle_radius = value[1]
        self.num_of_poses = value[2]

    def generate_circular_poses(self):
        target_list = []
        theta = 0
        delta_theta = 2*robodk.pi/self.num_of_poses
        origin_offset = self.ref_frame.Pose().Pos()
        circle_center = robodk.add3(origin_offset, [0, 0, self.circle_height])
        # set the first pose at the circle center
        pose_0 = robodk.Mat().eye() # initialize the pose with an identity matrix
        pose_0.setPos(circle_center)
        pose_0.setVZ([0,0,-1])
        pose_0.setVY([-1,0,0])
        pose_0.setVX([0,-1,0])
        target_list.append(pose_0)
        
        for i in range(self.num_of_poses + 1):
            pose = robodk.Mat().eye()
            radius_offset = [self.circle_radius*robodk.cos(theta), self.circle_radius*robodk.sin(theta), 0]
            target_position = robodk.add3(circle_center, radius_offset)
            target_z_axis = robodk.normalize3(robodk.subs3(origin_offset, target_position))
            target_x_axis, target_y_axis = PathPlanner.generate_pose_by_z_axis_while_constrain_x_axis_in_YZ_plane(target_z_axis)
            pose.setPos(target_position)
            pose.setVZ(target_z_axis)
            pose.setVY(target_y_axis)
            pose.setVX(target_x_axis)
            target_list.append(pose)
            # print(robodk.Pose_2_KUKA(pose))
            theta += delta_theta

        # the last pose
        target_list.append(pose_0)

        return target_list

    def generate_spherical_poses_by_tilt_and_swing_angles(self):
        # this function is obsolete
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
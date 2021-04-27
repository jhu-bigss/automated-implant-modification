import os, glob
import cv2
import numpy as np

def read_chessboard_image_from_dir(path, img_width, img_height, chessboard_width, chessboard_height, chessboard_square_size, calibrate_camera=False):
    # --------- Load the world poses ------------ #
    image_list = sorted(glob.glob(path + "*.jpg"))
    images = [cv2.imread(file) for file in image_list]
    c = ChessBoard(img_width, img_height, chessboard_width, chessboard_height, chessboard_square_size)

    # ---------- Camera calibration ---------- #
    if calibrate_camera:
        # calibrate(total_frame, time_btw_frame)
        c.calibrate(images, 300)
        return

    # ---------- Estimate the pose of calibration object w.r.t the camera ---------- #
    T_eye2world = c.estimate_pose(images)

    return T_eye2world

class ChessBoard:
    def __init__(self, image_width, image_height, num_of_width, num_of_height, size_of_square):

        self.image_width = image_width
        self.image_height = image_height
        self.num_of_width = num_of_width
        self.num_of_height = num_of_height
        self.size_of_square = size_of_square

        # Termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Prepare object points which had zero value w.r.t z-axis , like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((self.num_of_width * self.num_of_height, 3), np.float32)
        self.objp[:,:2] = np.mgrid[0:self.num_of_width, 0:self.num_of_height].T.reshape(-1, 2) * self.size_of_square

        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints = []  # 2d points in image plane.

    def calibrate(self, total_frame, time_btw_frame):
        # total_frame : total of the calibration frame
        # time_btw_frame(unit: milliseconds)

        num_of_frame = 1
        # If every image was already captured
        num_of_total_frame = len(total_frame)
        for frame in total_frame:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (self.num_of_width, self.num_of_height), None)
            # If found, add object points, image points (after refining them)
            if ret == True:
                self.objpoints.append(self.objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria) # increase accuracy
                self.imgpoints.append(corners2)
                # Draw and display the corners
                cv2.drawChessboardCorners(frame, (self.num_of_width, self.num_of_height), corners2, ret)
                cv2.putText(frame, 'Camera Calibration', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                            cv2.LINE_AA)
                cv2.putText(frame, str(num_of_frame) + '/' + str(num_of_total_frame), (50, 100), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (255, 0, 0), 2, cv2.LINE_AA)
                cv2.imshow('img', frame)
                cv2.waitKey(time_btw_frame)
                num_of_frame += 1
            if num_of_frame > num_of_total_frame:
                break
        # Calibrate
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, gray.shape[::-1], None, None)

        # Re-projection error
        mean_error = 0
        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        print("total error: {}".format(mean_error / len(self.objpoints)))
        # Save camera parameters
        print('Do you want to save the new result of camera calibration?(y/n)')
        cam_param_save_flag = input()
        if cam_param_save_flag == 'y':
            calib_file = cv2.FileStorage("camera_params.yaml", cv2.FILE_STORAGE_WRITE)
            calib_file.write("image_width", self.image_width)
            calib_file.write("image_height", self.image_height)
            calib_file.write("intrinsic", mtx)
            calib_file.write("dist_coeff", dist)
            calib_file.release()
            print('Camera Parameters saved: ' + os.getcwd() + '/camera_params.yaml')
        else:
            print('Finished. (did not save..)')

    def estimate_pose(self, caputred_image):
        # Check the coordinate of camera
        rect_width = 200
        rect_height = 150
        length_of_axis = 100 # 10 # 0.1
        interval_time = 10     # ms
        start_point = (int(self.image_width / 2 - rect_width / 2), int(self.image_height / 2 - rect_height / 2))
        end_point = (int(self.image_width / 2 + rect_width / 2), int(self.image_height / 2 + rect_height / 2))
        ref_pts = np.zeros((3, 1))

        # Load camera calibration
        calib_file = os.getcwd() + '/camera_params_handeye.yaml'
        calib_file = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
        mtx = calib_file.getNode("intrinsic").mat()
        dist = calib_file.getNode("dist_coeff").mat()

        def draw_axis(img, corners, imgpts):
            corner = tuple(corners[0].ravel())
            img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 3)
            img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 3)
            img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 3)
            return img

        axis = np.float32([[length_of_axis, 0, 0], [0, length_of_axis, 0], [0, 0, -length_of_axis]]).reshape(-1, 3)

        T_eye2world = np.zeros((1, 4, 4))
        image_cnt = 0
        for frame in caputred_image:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (self.num_of_width, self.num_of_height), None)
            if ret == True:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                ret, rvecs, tvecs = cv2.solvePnP(self.objp, corners2, mtx, dist)
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
                img = draw_axis(frame, corners2, imgpts)
                print('Load image', image_cnt)
                image_cnt += 1
                cv2.imshow('img', img)
                cv2.waitKey(interval_time)
                R_eye2world = np.empty((3, 3))
                cv2.Rodrigues(rvecs, R_eye2world) # Convert rotation vector to a rotation matrix
                T_eye2world_ = np.concatenate((R_eye2world, tvecs), axis=1)
                T_eye2world_ = np.concatenate((T_eye2world_, np.array([[0, 0, 0, 1]])), axis=0)

                T_eye2world = np.concatenate((T_eye2world, np.expand_dims(T_eye2world_, axis=0)), axis=0)

        return T_eye2world[1:]

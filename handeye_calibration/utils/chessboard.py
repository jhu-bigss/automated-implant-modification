import os
import cv2
import numpy as np

DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_CHESSBOARD_WIDTH = 8
DEFAULT_CHESSBOARD_HEIGHT = 7
DEFAULT_CHESSBOARD_SQUARE_SIZE = 30 # mm

class ChessBoard:
    def __init__(self, image_width=DEFAULT_WIDTH, image_height=DEFAULT_HEIGHT, num_of_width=DEFAULT_CHESSBOARD_WIDTH, num_of_height=DEFAULT_CHESSBOARD_HEIGHT, size_of_square=DEFAULT_CHESSBOARD_SQUARE_SIZE):

        self.image_width = image_width
        self.image_height = image_height
        self.num_of_width = num_of_width
        self.num_of_height = num_of_height
        self.size_of_square = size_of_square

        # Termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Prepare object points which had zero value w.r.t z-axis , like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((self.num_of_width * self.num_of_height, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.num_of_width, 0:self.num_of_height].T.reshape(-1, 2) * self.size_of_square

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
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
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
        cam_param_flag = input()
        if cam_param_flag == 'y':
            np.savez('data/cam_parameter', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
            print('Saved.')
        else:
            print('Finished. (did not save..)')

    def estimate_pose(self, caputred_image):
        # Check the coordinate of camera
        rect_width = 200
        rect_height = 150
        length_of_axis = 0.1 # 10 # 0.1
        interval_time = 10     # ms
        start_point = (int(self.image_width / 2 - rect_width / 2), int(self.image_height / 2 - rect_height / 2))
        end_point = (int(self.image_width / 2 + rect_width / 2), int(self.image_height / 2 + rect_height / 2))
        ref_pts = np.zeros((3, 1))

        cam_calibration_file = 'data/cam_parameter.npz'
        # Load previously saved data
        with np.load(cam_calibration_file) as X:
            mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]
            print('Camera parameter\n', mtx)

        def draw(img, corners, imgpts):
            corner = tuple(corners[0].ravel())
            img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 3)
            img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 3)
            img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 3)
            return img

        axis = np.float32([[length_of_axis, 0, 0], [0, length_of_axis, 0], [0, 0, -length_of_axis]]).reshape(-1, 3)

        print('Load captured images')
        T_eye2world = np.zeros((1, 4, 4))
        image_cnt = 0
        for frame in caputred_image:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (self.num_of_width, self.num_of_height), None)
            if ret == True:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                ret, rvecs, tvecs = cv2.solvePnP(self.objp, corners2, mtx, dist)
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
                img = draw(frame, corners2, imgpts)
                print('Load', image_cnt, 'th image.' )
                image_cnt += 1
                cv2.imshow('img', img)
                cv2.waitKey(interval_time)
                R_eye2world = np.empty((3, 3))
                cv2.Rodrigues(rvecs, R_eye2world)
                # T_eye2world_ = kin.homogMatfromRotAndTrans(R_eye2world, tvecs)
                dummy = np.array([[0, 0, 0, 1]])
                T_eye2world_ = np.concatenate((R_eye2world, tvecs), axis=1)
                T_eye2world_ = np.concatenate((T_eye2world_, dummy), axis=0)

                T_eye2world = np.concatenate((T_eye2world, np.expand_dims(T_eye2world_, axis=0)), axis=0)

        return T_eye2world[1:]

    
if __name__ == "__main__":

    c = ChessBoard()
    c.estimate_pose(1)
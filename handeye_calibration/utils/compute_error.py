
import numpy as np
import matplotlib.pyplot as plt
from utils import kinematics as kin

def woGroundTruth(A, B, X_est, R_tol):
    """woGroundTruth.

    :param A: Numpy array (n, 4, 4), N set of transformation matrix from robot base to robot hand 
    :param B: Numpy array (n, 4, 4), N set of transformation matrix from camera to world 
    :param X_est: Numpy array (total method, 4, 4), stimation set of transformation matrix from robot hand to robot eye
    :param R_tol: Scalar, Tolerance of rotation matrix condition
    :return pos_err, ori_err: List array (total method, ), (total method, ), Position and orientation(deg) error at the each method
    """

    n = len(A)
    cnt_error = 0
    A_rel = np.empty((1, 4, 4))
    B_rel = np.empty((1, 4, 4))
    for A_1, B_1 in zip(A, B):
        n -= 1
        for i in range(n):
            A_2 = A[-i-1]
            B_2 = B[-i-1]
            A_rel_ = np.dot(kin.homogeneousInverse(A_2), A_1)
            B_rel_ = np.dot(B_2, kin.homogeneousInverse(B_1))
            A_rel = np.concatenate((A_rel, np.expand_dims(A_rel_, axis=0)), axis=0)
            B_rel = np.concatenate((B_rel, np.expand_dims(B_rel_, axis=0)), axis=0)
            cnt_error += 1

    pos_e = []
    ori_e = []
    for X_est_ in X_est:
        pos_err = []
        ori_err = []
        for A_rel_, B_rel_ in zip(A_rel[1:], B_rel[1:]):
            # Position
            transl_error = (np.dot(kin.rotationFromHmgMatrix(A_rel_), kin.translationFromHmgMatrix(X_est_)) + kin.translationFromHmgMatrix(A_rel_)) \
                           - (np.dot(kin.rotationFromHmgMatrix(X_est_), kin.translationFromHmgMatrix(B_rel_)) + kin.translationFromHmgMatrix(X_est_))
            # Orientation
            AX = np.dot(kin.rotationFromHmgMatrix(A_rel_), kin.rotationFromHmgMatrix(X_est_))
            XB = np.dot(kin.rotationFromHmgMatrix(X_est_), kin.rotationFromHmgMatrix(B_rel_))
            rot_error = np.dot(np.linalg.inv(AX), XB)
            #_, angular_error = kin.rotMatrixToRodVector(rot_error, R_tol) # 1
            angular_error = kin.orientationErrorByRotation(AX, XB) # 2

            pos_err.append(np.linalg.norm(transl_error))
            ori_err.append(angular_error)

        pos_e_, ori_e_ = np.mean(pos_err), np.mean(ori_err)
        pos_e.append(pos_e_)
        ori_e.append(ori_e_)
    return np.array(pos_e), np.array(ori_e)


def withGroundTruth(X_true, X_est, R_tol):
    """withGroundTruth.
    Only simulation (not real experiment!)

    'N' is the number of total methods
    :param X_true: Numpy array (n, 4, 4), N ground truth set of transformation matrix from robot hand to robot eye
    :param X_est: Numpy array (n, 4, 4), N estimation set of transformation matrix from robot hand to robot eye
    :param R_tol: Scalar, Tolerance of rotation matrix condition
    :return pos_err, ori_err: (n,), (n,), N set of position and orientation(deg) error
    """
    pos_err = []
    ori_err = []
    for X_est_ in X_est:
        pos_err_ = np.linalg.norm(np.subtract(X_true[:3, 3], X_est_[:3, 3])) / np.linalg.norm(X_true[:3, 3])

        # _, rvec_diff__ = kin.rotMatrixToRodVector(np.dot(np.transpose(R_hand2eye_est), R_hand2eye_true))
        
        #ori_err_ = np.linalg.norm(X_true[0:3, 0:3] - X_est_[0:3, 0:3]) # Frobenius norm
        ori_err_ = np.dot(X_true[:3, :3].T, X_est_[:3, :3])

        # Rotation matrix -> Euler Angle
        # ori_err_ : Norm of Euler angle(deg)
        ori_err_ = np.linalg.norm(np.rad2deg(kin.rotMatrixToEuler(ori_err_, R_tol)))
        pos_err.append(pos_err_)
        ori_err.append(ori_err_)
    return np.array(pos_err), np.array(ori_err)


def graphPlot(xlabel_data, pos_e, ori_e, ex_method, observ):
    """graphPlot.

    :param pos_e: Numpy array, (n, total_method), Position error
    :param ori_e: Numpy array, (n, total_method), Orientation error
    :param ex_method:
    :param observ: Scalar(1, 2, 3), 1: Select the label, 1: The number of poses, 2: Distance between poses, 3: Noise
    """

    leg_method = ['TSA', 'PAR', 'HOR', 'AND', 'DAN', 'AXC1', 'AXC2', 'HORN',
            'OURS', 'LI', 'SHA', 'TZC1', 'TZC2']
    if ex_method:
        for i in ex_method:
            if i == 1:
                leg_method.remove('TSA')
            elif i == 2:
                leg_method.remove('PAR')
            elif i == 3:
                leg_method.remove('HOR')
            elif i == 4:
                leg_method.remove('AND')
            elif i == 5:
                leg_method.remove('DAN')
            elif i == 6:
                leg_method.remove('AXC1')
            elif i == 7:
                leg_method.remove('AXC2')
            elif i == 8:
                leg_method.remove('HORN')
            elif i == 9:
                leg_method.remove('OURS')
            elif i == 10:
                leg_method.remove('LI')
            elif i == 11:
                leg_method.remove('SHA')
            elif i == 12:
                leg_method.remove('TZC1')
            elif i == 13:
                leg_method.remove('TZC2')

    fig_error = plt.figure(figsize=(12, 12))
    plt_pos = fig_error.add_subplot(121)
    plt_ori = fig_error.add_subplot(122)
    leg_pos = plt_pos.plot(xlabel_data, pos_e)
    leg_ori = plt_ori.plot(xlabel_data, ori_e)
    plt_pos.set_title('Position error between ground truth and estimation')
    plt_ori.set_title('Orientation error between ground truth and estimation')
    plt_pos.legend(handles=leg_pos, labels=(leg_method))

    if observ == 1:
        plt_pos.set_xlabel('The number of poses')
    elif observ == 2:
        plt_pos.set_xlabel('Distance between poses')
    elif observ == 3:
        plt_pos.set_xlabel('Gaussian noise')
    plt_pos.grid(True, linestyle='--')
    plt_ori.grid(True, linestyle='--')
    plt.show()

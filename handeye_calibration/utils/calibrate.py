import cv2
from .axxb import Tsai, Park, Horaud, Andreff, Daniilidis, Ali_xc1, Ali_xc2, Horaud_non, e2
from .axyb import li, shah, tabb_zc1, tabb_zc2
import numpy as np

def axxb_method(A, B, lm_tol, ex_method):
    """axxb_method.

    :param A: Numpy array, (total_poses, 4, 4), Transformation matrix from robot base to robot hand
    :param B: Numpy array, (total_poses, 4, 4), Transformation matrix from camera to target object
    :param lm_tol: Scalar, Tolerance using levenberg-marquardt method in scipy.least_sq
    :param ex_method: list, (n,), Excluding method, choices = 1, 2, 3, 4, 5, 6, 7, 8, 9
    :return X: Numpy array, (total_methods, 4, 4)
    """
    axxb_method = list(range(1, 10))
    for i in ex_method:
        try:
            axxb_method.remove(i)
        except ValueError:
            pass

    X0 = np.ones((4, 4))
    X = np.empty((1, 4, 4))
    for method in axxb_method:
        if method == 1:
            X_Tsai, time_Tsai = Tsai(A, B)
            X = np.concatenate((X, np.expand_dims(X_Tsai, axis=0)), axis=0)
        elif method == 2:
            X_Park, time_Park = Park(A, B)
            X = np.concatenate((X, np.expand_dims(X_Park, axis=0)), axis=0)
        elif method == 3:
            X_Horaud, time_Horaud = Horaud(A, B)
            X = np.concatenate((X, np.expand_dims(X_Horaud, axis=0)), axis=0)
        elif method == 4:
            X_Andreff, time_Andreff = Andreff(A, B)
            X = np.concatenate((X, np.expand_dims(X_Andreff, axis=0)), axis=0)
        elif method == 5:
            X_Daniilidis, time_Daniilidis = Daniilidis(A, B)
            X = np.concatenate((X, np.expand_dims(X_Daniilidis, axis=0)), axis=0)
        elif method == 6:
            X0 = X_Tsai # Initial
            X_Ali_xc1, time_Ali_xc1 = Ali_xc1(A, B, X0, lm_tol)
            X = np.concatenate((X, np.expand_dims(X_Ali_xc1, axis=0)), axis=0)
        elif method == 7:
            X0 = X_Tsai  # Initial
            #X0 = X_Park # Initial
            X_Ali_xc2, time_Ali_xc2 = Ali_xc2(A, B, X0, lm_tol)
            X = np.concatenate((X, np.expand_dims(X_Ali_xc2, axis=0)), axis=0)
        elif method == 8:
            X0 = X_Tsai # Initial
            #X0 = X_Park # Initial
            X_Horaud_non, time_Horaud_non = Horaud_non(A, B, X0, lm_tol)
            X = np.concatenate((X, np.expand_dims(X_Horaud_non, axis=0)), axis=0)
        elif method == 9:
            X0 = X_Tsai # Initial
            X_e2, time_e2 = e2(A, B, X0)
            X = np.concatenate((X, np.expand_dims(X_e2, axis=0)), axis=0)

    return X[1:]


def axyb_method(A, B, lm_tol, ex_method):
    """axyb_method.

    :param A: Numpy array, (total_poses, 4, 4), Transformation matrix from robot base to robot hand
    :param B: Numpy array, (total_poses, 4, 4), Transformation matrix from camera to target object
    :param lm_tol: Scalar, Tolerance using levenberg-marquardt method in scipy.least_sq
    :param ex_method: list, (n,), Excluding method, choices = 10, 11, 12, 13
    :return X: Numpy array, (total_methods, 4, 4)
    :return Y: Numpy array, (total_methods, 4, 4)
    """
    axyb_method = list(range(10, 14))
    for i in ex_method:
        try:
            axyb_method.remove(i)
        except ValueError:
            pass
    
    X0, Y0 = np.empty((4, 4)), np.empty((4, 4))
    X = np.empty((1, 4, 4))
    Y = np.empty((1, 4, 4))
    for method in axyb_method:
        if method == 10:
            X_Li, Y_Li, time_Li = li(A, B)
            X = np.concatenate((X, np.expand_dims(X_Li, axis=0)), axis=0)
            Y = np.concatenate((Y, np.expand_dims(Y_Li, axis=0)), axis=0)
        elif method == 11:
            X_Shah, Y_Shah, time_Shah = shah(A, B)
            X = np.concatenate((X, np.expand_dims(X_Shah, axis=0)), axis=0)
            Y = np.concatenate((Y, np.expand_dims(Y_Shah, axis=0)), axis=0)
        elif method == 12:
            X0, Y0 = X_Li, Y_Li # Initial matrix
            X_Tabb_zc1, Y_Tabb_zc1, time_Tabb_zc1 = tabb_zc1(A, B, X0, Y0, lm_tol)
            X = np.concatenate((X, np.expand_dims(X_Tabb_zc1, axis=0)), axis=0)
            Y = np.concatenate((Y, np.expand_dims(Y_Tabb_zc1, axis=0)), axis=0)
        elif method == 13:
            X0, Y0 = X_Li, Y_Li # Initial matrix
            X_Tabb_zc2, Y_Tabb_zc2, time_Tabb_zc2 = tabb_zc2(A, B, X0, Y0, lm_tol)
            X = np.concatenate((X, np.expand_dims(X_Tabb_zc2, axis=0)), axis=0)
            Y = np.concatenate((Y, np.expand_dims(Y_Tabb_zc2, axis=0)), axis=0)
    
    return X[1:], Y[1:]

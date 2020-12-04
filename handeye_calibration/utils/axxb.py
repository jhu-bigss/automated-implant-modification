"""
AX=XB
1. Tsai
2. Park
3. Horaud
4. Andreff
5. Daniilidis
6. Ali(Xc1)
7. Ali(Xc2)
8. Horaud(Nonlinear)
9. Proposed

"""
import cv2
import numpy as np
from . import kinematics as kin
import time
import scipy.optimize
from pytransform3d.rotations import *

def Tsai_rank(A, B):
    """Tsai.

    :param A: Numpy array (n, 4, 4), N set of transformation matrix from robot base to robot hand 
    :param B: Numpy array (n, 4, 4), N set of transformation matrix from camera to world 
    :return X: Numpy array (4, 4), Transformation matrix from robot hand to camera 
    :return elapsed_time: Elapsed time
    """

    n = len(A)
    M = np.empty((1, 3))
    N = np.empty((1, 1))

    # --- To get the relative A and B --- #
    for Ta_1, Tb_1 in zip(A, B):
        n -= 1
        for i in range(n):
            Ta_2 = A[-i - 1]
            Tb_2 = B[-i - 1]
            Ta_ = np.dot(kin.homogeneousInverse(Ta_2), Ta_1)
            Pa = 2 * kin.rot2quatMinimal(kin.rotationFromHmgMatrix(Ta_))

            Tb_ = np.dot(Tb_2, kin.homogeneousInverse(Tb_1))
            Pb = 2 * kin.rot2quatMinimal(kin.rotationFromHmgMatrix(Tb_))

            M_ = kin.skew(Pa + Pb)
            N_ = np.reshape(Pb - Pa, (3, 1))
            
            M = np.concatenate((M, M_), axis=0) 
            N = np.concatenate((N, N_), axis=0) 

    M = M[1:]
    N = N[1:]
    M_rank = np.linalg.matrix_rank(M)
    N_rank = np.linalg.matrix_rank(N)

    return M_rank, N_rank


def Tsai(A, B):
    """Tsai.

    :param A: Numpy array (n, 4, 4), N set of transformation matrix from robot base to robot hand 
    :param B: Numpy array (n, 4, 4), N set of transformation matrix from camera to world 
    :return X: Numpy array (4, 4), Transformation matrix from robot hand to camera 
    :return elapsed_time: Elapsed time
    """
    tic = time.perf_counter()
    Ra = A[:, :3, :3]
    ta = A[:, :3, 3]
    ta = np.expand_dims(ta, axis=2)
    Rb = B[:, :3, :3]
    tb = B[:, :3, 3]
    tb = np.expand_dims(tb, axis=2)
    Rx = np.empty((3, 3))
    tx = np.empty((3, 1))
    cv2.calibrateHandEye(Ra, ta, Rb, tb, Rx, tx, method=0)
    toc = time.perf_counter()
    elapsed_time = toc - tic

    X = kin.homogMatfromRotAndTrans(Rx, tx)
    return X, elapsed_time


def Park(A, B):
    """Park.

    :param A: Numpy array (n,4,4), N set of transformation matrix from robot base to robot hand 
    :param B: Numpy array (n,4,4), N set of transformation matrix from camera to world 
    :return X: Numpy array (4, 4), Transformation matrix from robot hand to camera 
    :return elapsed_time: Elapsed time
    """
    tic = time.perf_counter()
    Ra = A[:, :3, :3]
    ta = A[:, :3, 3]
    Rb = B[:, :3, :3]
    tb = B[:, :3, 3]
    Rx = np.empty((3, 3))
    tx = np.empty((3, 1))
    cv2.calibrateHandEye(Ra, ta, Rb, tb, Rx, tx, method=1)
    toc = time.perf_counter()
    elapsed_time = toc - tic
    X = kin.homogMatfromRotAndTrans(Rx, tx)
    return X, elapsed_time


def Horaud(A, B):
    """Horaud.

    :param A: Numpy array (n,4,4), N set of transformation matrix from robot base to robot hand 
    :param B: Numpy array (n,4,4), N set of transformation matrix from camera to world 
    :return X: Numpy array (4, 4), Transformation matrix from robot hand to camera 
    :return elapsed_time: Elapsed time
    """
    tic = time.perf_counter()
    Ra = A[:, :3, :3]
    ta = A[:, :3, 3]
    Rb = B[:, :3, :3]
    tb = B[:, :3, 3]
    Rx = np.empty((3, 3))
    tx = np.empty((3, 1))
    cv2.calibrateHandEye(Ra, ta, Rb, tb, Rx, tx, method=2)
    toc = time.perf_counter()
    elapsed_time = toc - tic
    X = kin.homogMatfromRotAndTrans(Rx, tx)
    return X, elapsed_time


def Andreff(A, B):
    """Andreff.

    :param A: Numpy array (n,4,4), N set of transformation matrix from robot base to robot hand 
    :param B: Numpy array (n,4,4), N set of transformation matrix from camera to world 
    :return X: Numpy array (4, 4), Transformation matrix from robot hand to camera 
    :return elapsed_time: Elapsed time
    """
    tic = time.perf_counter()

    Ra = A[:, :3, :3]
    ta = A[:, :3, 3]
    Rb = B[:, :3, :3]
    tb = B[:, :3, 3]
    Rx = np.empty((3, 3))
    tx = np.empty((3, 1))
    cv2.calibrateHandEye(Ra, ta, Rb, tb, Rx, tx, method=3)

#    n = len(A)
#    F = np.empty((1, 12))
#    G = np.empty((1, 1))
#    # --- To get the relative A and B --- #
#    for Ta_1, Tb_1 in zip(A, B):
#        n -= 1
#        for i in range(n):
#            Ta_2 = A[-i - 1]
#            Tb_2 = B[-i - 1]
#            Ta = np.dot(kin.homogeneousInverse(Ta_2), Ta_1)
#            Tb = np.dot(Tb_2, kin.homogeneousInverse(Tb_1))
#
#            f00 = np.eye(9) - np.kron(kin.rotationFromHmgMatrix(Ta), kin.rotationFromHmgMatrix(Tb))
#            f01 = np.zeros((9, 3))
#            f10 = np.kron(np.eye(3), kin.translationFromHmgMatrix(Tb).T)
#            f11 = np.eye(3) - kin.rotationFromHmgMatrix(Ta)
#            f0 = np.concatenate((f00, f01), axis=1)
#            f1 = np.concatenate((f10, f11), axis=1)
#            f = np.concatenate((f0, f1), axis=0)
#            g = np.concatenate((np.zeros((9, 1)), kin.translationFromHmgMatrix(Ta)), axis=0)
#
#            F = np.concatenate((F, f), axis=0)
#            G = np.concatenate((G, g), axis=0)
#
#    X = np.empty((12, 1))
#    cv2.solve(F[1:], G[1:], X, flags=cv2.DECOMP_SVD)
#    Rx = np.reshape(X[:9], (3, 3))
#    Rx = kin.normalizeRotation(Rx)
#    tx = X[9:]

    toc = time.perf_counter()
    elapsed_time = toc - tic
    X = kin.homogMatfromRotAndTrans(Rx, tx)
    return X, elapsed_time


def Daniilidis(A, B):
    """Daniilidis.

    :param A: Numpy array (n,4,4), N set of transformation matrix from robot base to robot hand 
    :param B: Numpy array (n,4,4), N set of transformation matrix from camera to world 
    :return X: Numpy array (4, 4), Transformation matrix from robot hand to camera 
    :return elapsed_time: Elapsed time
    """
    tic = time.perf_counter()
    Ra = A[:, :3, :3]
    ta = A[:, :3, 3]
    Rb = B[:, :3, :3]
    tb = B[:, :3, 3]
    Rx = np.empty((3, 3))
    tx = np.empty((3, 1))
    cv2.calibrateHandEye(Ra, ta, Rb, tb, Rx, tx, method=4)
    toc = time.perf_counter()
    elapsed_time = toc - tic
    X = kin.homogMatfromRotAndTrans(Rx, tx)
    return X, elapsed_time


def Ali_xc1(A, B, X0, cost_fn_tol):
    """Ali_xc1.

    :param A: Numpy array (n,4,4), N set of transformation matrix from robot base to robot hand 
    :param B: Numpy array (n,4,4), N set of transformation matrix from camera to world 
    :param X0: Numpy array (4, 4), Initial value for nonlinear method
    :return X: Numpy array (4, 4), Transformation matrix from robot hand to camera 
    :return elapsed_time: Elapsed time
    """
    tic = time.perf_counter()
    n = len(A)
    Ta_rel = np.empty((1, 4, 4))
    Tb_rel = np.empty((1, 4, 4))

    # --- To get the relative A and B --- #
    for Ta_1, Tb_1 in zip(A, B):
        n -= 1
        for i in range(n):
            Ta_2 = A[-i - 1]
            Tb_2 = B[-i - 1]
            Ta_ = np.dot(kin.homogeneousInverse(Ta_2), Ta_1)
            Tb_ = np.dot(Tb_2, kin.homogeneousInverse(Tb_1))
            Ta_rel = np.concatenate((Ta_rel, np.expand_dims(Ta_, axis=0)), axis=0)
            Tb_rel = np.concatenate((Tb_rel, np.expand_dims(Tb_, axis=0)), axis=0)
    Ta_rel = Ta_rel[1:]
    Tb_rel = Tb_rel[1:]

    def lossfn(x):
        cost = []
        X = kin.homogMatfromQuatAndTrans(x[0:4], x[4:7])
        for Ta_, Tb_ in zip(Ta_rel, Tb_rel):
            err = np.dot(Ta_, X) - np.dot(X, Tb_)
            ori_e = np.linalg.norm(kin.rotationFromHmgMatrix(err)) # Frobenius norm 
            pos_e = np.linalg.norm(kin.translationFromHmgMatrix(err))
            cost_ = ori_e + pos_e
            cost.append(cost_)
        return cost

    Rx0 = kin.rotationFromHmgMatrix(X0)
    tx0 = kin.translationFromHmgMatrix(X0)
    x0 = np.concatenate((quaternion_from_matrix(Rx0), tx0.reshape(3)), axis=0)

    ''' lm method
    - jac: only '2-point'
    - ftol: Tolerance for termination by the change of the cost function. Default is 1e-8.
      xtol: Tolerance for termination by the change of the independent variables. Default is 1e-8. 
      gtol: Tolerance for termination by the norm of the gradient. Default is 1e-8. 
    If None, the termination by this condition is disabled.
    - loss: only 'linear' option, so 'f_scale' also cannot control.
    '''
    
    res = scipy.optimize.least_squares(fun=lossfn, x0=x0, method='lm', ftol=cost_fn_tol)
    X = kin.homogMatfromQuatAndTrans(res.x[0:4], res.x[4:7])
    toc = time.perf_counter()
    elapsed_time = toc - tic
#    print('Time(Ali_xc1):', round(elapsed_time, 2))
    return X, elapsed_time


def Ali_xc2(A, B, X0, cost_fn_tol):
    """Ali_xc2.

    :param A: Numpy array (n,4,4), N set of transformation matrix from robot base to robot hand 
    :param B: Numpy array (n,4,4), N set of transformation matrix from camera to world 
    :param X0: Numpy array (4, 4), Initial value for nonlinear method
    :return X: Numpy array (4, 4), Transformation matrix from robot hand to camera 
    :return elapsed_time: Elapsed time
    """
    tic = time.perf_counter()
    n = len(A)
    Ta_rel = np.empty((1, 4, 4))
    Tb_rel = np.empty((1, 4, 4))

    # --- To get the relative A and B --- #
    for Ta_1, Tb_1 in zip(A, B):
        n -= 1
        for i in range(n):
            Ta_2 = A[-i - 1]
            Tb_2 = B[-i - 1]
            Ta_ = np.dot(kin.homogeneousInverse(Ta_2), Ta_1)
            Tb_ = np.dot(Tb_2, kin.homogeneousInverse(Tb_1))
            Ta_rel = np.concatenate((Ta_rel, np.expand_dims(Ta_, axis=0)), axis=0)
            Tb_rel = np.concatenate((Tb_rel, np.expand_dims(Tb_, axis=0)), axis=0)
    Ta_rel = Ta_rel[1:]
    Tb_rel = Tb_rel[1:]

    def lossfn(x):
        cost = []
        X = kin.homogMatfromQuatAndTrans(x[0:4], x[4:7])
        for Ta_, Tb_ in zip(Ta_rel, Tb_rel):
            err = Ta_ - np.dot(np.dot(X, Tb_), kin.homogeneousInverse(X))
            ori_e = np.linalg.norm(kin.rotationFromHmgMatrix(err)) # Frobenius norm 
            pos_e = np.linalg.norm(kin.translationFromHmgMatrix(err))
            cost_ = ori_e + pos_e
            cost.append(cost_)
        return cost

    Rx0 = kin.rotationFromHmgMatrix(X0)
    tx0 = kin.translationFromHmgMatrix(X0)
    x0 = np.concatenate((quaternion_from_matrix(Rx0), tx0.reshape(3)), axis=0)

    ''' lm method
    - jac: only '2-point'
    - ftol: Tolerance for termination by the change of the cost function. Default is 1e-8.
      xtol: Tolerance for termination by the change of the independent variables. Default is 1e-8. 
      gtol: Tolerance for termination by the norm of the gradient. Default is 1e-8. 
    If None, the termination by this condition is disabled.
    - loss: only 'linear' option, so 'f_scale' also cannot control.
    '''
    
    res = scipy.optimize.least_squares(fun=lossfn, x0=x0, method='lm', ftol=cost_fn_tol)
    X = kin.homogMatfromQuatAndTrans(res.x[0:4], res.x[4:7])
    toc = time.perf_counter()
    elapsed_time = toc - tic
#    print('Time(Ali_xc2):', round(elapsed_time, 2))
    return X, elapsed_time


def Horaud_non(A, B, X0, cost_fn_tol):
    """Horaud_non.

    :param A: Numpy array (n,4,4), N set of transformation matrix from robot base to robot hand 
    :param B: Numpy array (n,4,4), N set of transformation matrix from camera to world 
    :param X0: Numpy array (4, 4), Initial value for nonlinear method
    :return X: Numpy array (4, 4), Transformation matrix from robot hand to camera 
    :return elapsed_time: Elapsed time
    """

    tic = time.perf_counter()
    n = len(A)
    lmd = [1, 1, 1000]
    Ta_rel = np.empty((1, 4, 4))
    Tb_rel = np.empty((1, 4, 4))

    # --- To get the relative A and B --- #
    for Ta_1, Tb_1 in zip(A, B):
        n -= 1
        for i in range(n):
            Ta_2 = A[-i - 1]
            Tb_2 = B[-i - 1]
            Ta_ = np.dot(kin.homogeneousInverse(Ta_2), Ta_1)
            Tb_ = np.dot(Tb_2, kin.homogeneousInverse(Tb_1))
            Ta_rel = np.concatenate((Ta_rel, np.expand_dims(Ta_, axis=0)), axis=0)
            Tb_rel = np.concatenate((Tb_rel, np.expand_dims(Tb_, axis=0)), axis=0)
    Ta_rel = Ta_rel[1:]
    Tb_rel = Tb_rel[1:]

    def lossfn(x):
        constraint = np.linalg.norm(x[0:4]) - 1     # Unit-quaternion
        Rx = matrix_from_quaternion(x[0:4])
        tx = x[4:7]
        cost = []
        for Ta_, Tb_ in zip(Ta_rel, Tb_rel):
            Ra_ = kin.rotationFromHmgMatrix(Ta_)
            Rb_ = kin.rotationFromHmgMatrix(Tb_)
            ta_ = kin.translationFromHmgMatrix(Ta_)
            tb_ = kin.translationFromHmgMatrix(Tb_)
            ## ---- Orientation Error 1 ---- ## 
            # --- Frobenius norm --- #
            Rax_xb = (np.dot(Ra_, Rx) - np.dot(Rx, Rb_))
            err1_ = np.linalg.norm(Rax_xb)
            # --- Nuclear norm --- #
            #_, s, _  = np.linalg.svd(Rax_xb)
            #err1_ = np.linalg.norm(s)

            ## ---- Orientation Error 2 ---- ## 
            #Rax = np.dot(Ra_, Rx)
            #Rxb = np.dot(Rx, Rb_)
            #err1_ = kin.orientationErrorByRotation(Rax, Rxb)
            #err1_vec = np.empty((3, 1))
            #cv2.Rodrigues(np.dot(Rax, Rxb), err1_vec)
            #err1_ = np.linalg.norm(err1_vec)

            err2_ = np.linalg.norm(Ra_.dot(tx) + ta_ - Rx.dot(tb_) - tx)      # Ra_ * tx + ta = Rx * tb + tx
            cost_ = lmd[0] * err1_ + lmd[1] * err2_ + lmd[2] * constraint
            cost.append(cost_)
        return cost

    Rx0 = kin.rotationFromHmgMatrix(X0)
    tx0 = kin.translationFromHmgMatrix(X0)
    x0 = np.concatenate((quaternion_from_matrix(Rx0), tx0.reshape(3)), axis=0)
    res = scipy.optimize.least_squares(fun=lossfn, x0=x0, method='lm', ftol=cost_fn_tol)
    X = kin.homogMatfromQuatAndTrans(res.x[0:4], res.x[4:7])
    toc = time.perf_counter()
    elapsed_time = toc - tic
#    print('Time(Horaud_nonlinear):', round(elapsed_time, 2))
    return X, elapsed_time


def e1(A, B, X0):
    """e1.

    :param A:
    :param B:
    :param X:
    """
    tic = time.perf_counter()
    n = len(A)
    lmd = [1, 1, 100]
#    n = len(Ra)
#    Ta = np.concatenate((Ra, ta), axis=2)
#    dummy = np.expand_dims(np.vstack([[0, 0, 0, 1]] * n), axis=1)
#    Ta = np.concatenate((Ta, dummy), axis=1)
#    Tb = np.concatenate((Rb, tb), axis=2)
#    Tb = np.concatenate((Tb, dummy), axis=1)
    Ta_rel = np.empty((1, 4, 4))
    Tb_rel = np.empty((1, 4, 4))

    # --- To get the relative A and B --- #
    for Ta_1, Tb_1 in zip(A, B):
        n -= 1
        for i in range(n):
            Ta_2 = A[-i - 1]
            Tb_2 = B[-i - 1]
            Ta_ = np.dot(kin.homogeneousInverse(Ta_2), Ta_1)
            Tb_ = np.dot(Tb_2, kin.homogeneousInverse(Tb_1))
            Ta_rel = np.concatenate((Ta_rel, np.expand_dims(Ta_, axis=0)), axis=0)
            Tb_rel = np.concatenate((Tb_rel, np.expand_dims(Tb_, axis=0)), axis=0)
    Ta_rel = Ta_rel[1:]
    Tb_rel = Tb_rel[1:]

    def lossfn(x):
        constraint = np.linalg.norm(x[0:4]) - 1     # Unit-quaternion
        Rx = matrix_from_quaternion(x[0:4])
        tx = x[4:7]
        cost = []
        for Ta_, Tb_ in zip(Ta_rel, Tb_rel):
            Ra_ = kin.rotationFromHmgMatrix(Ta_)
            Rb_ = kin.rotationFromHmgMatrix(Tb_)
            ta_ = kin.translationFromHmgMatrix(Ta_)
            tb_ = kin.translationFromHmgMatrix(Tb_)
            ## ---- Orientation Error 1 ---- ## 
            # --- Frobenius norm --- #
            Rax_xb = (np.dot(Ra_, Rx) - np.dot(Rx, Rb_))
            err1_ = np.linalg.norm(Rax_xb)
            # --- Nuclear norm --- #
            #_, s, _  = np.linalg.svd(Rax_xb)
            #err1_ = np.linalg.norm(s)

            ## ---- Orientation Error 2 ---- ## 
            #Rax = np.dot(Ra_, Rx)
            #Rxb = np.dot(Rx, Rb_)
            #err1_ = kin.orientationErrorByRotation(Rax, Rxb)
            #err1_vec = np.empty((3, 1))
            #cv2.Rodrigues(np.dot(Rax, Rxb), err1_vec)
            #err1_ = np.linalg.norm(err1_vec)

            err2_ = np.linalg.norm(Ra_.dot(tx) + ta_ - Rx.dot(tb_) - tx)      # Ra_ * tx + ta = Rx * tb + tx
            cost_ = lmd[0] * err1_ + lmd[1] * err2_ + lmd[2] * constraint
            cost.append(cost_)
        return cost

    Rx0 = kin.rotationFromHmgMatrix(X0)
    tx0 = kin.translationFromHmgMatrix(X0)
    x0 = np.concatenate((quaternion_from_matrix(Rx0), tx0.reshape(3)), axis=0)
    res = scipy.optimize.least_squares(fun=lossfn, x0=x0, method='lm')
    X = kin.homogMatfromQuatAndTrans(res.x[0:4], res.x[4:7])
    toc = time.perf_counter()
    elapsed_time = toc - tic
    return X, elapsed_time


def e2(A, B, X0):
    """e1.

    :param A:
    :param B:
    :param X:
    """
    tic = time.perf_counter()
    n = len(A)
    lmd = [1, 10000]
    Ta_rel = np.empty((1, 4, 4))
    Tb_rel = np.empty((1, 4, 4))

    # --- To get the relative A and B --- #
    for Ta_1, Tb_1 in zip(A, B):
        n -= 1
        for i in range(n):
            Ta_2 = A[-i - 1]
            Tb_2 = B[-i - 1]
            Ta_ = np.dot(kin.homogeneousInverse(Ta_2), Ta_1)
            Tb_ = np.dot(Tb_2, kin.homogeneousInverse(Tb_1))
            Ta_rel = np.concatenate((Ta_rel, np.expand_dims(Ta_, axis=0)), axis=0)
            Tb_rel = np.concatenate((Tb_rel, np.expand_dims(Tb_, axis=0)), axis=0)
    Ta_rel = Ta_rel[1:]
    Tb_rel = Tb_rel[1:]

    def lossfn(x):
        constraint = np.linalg.norm(x[0:4]) - 1     # Unit-quaternion
        Rx = matrix_from_quaternion(x[0:4])
        tx = x[4:7]
        cost = []
        for Ta_, Tb_ in zip(Ta_rel, Tb_rel):
            Ra_ = kin.rotationFromHmgMatrix(Ta_)
            Rb_ = kin.rotationFromHmgMatrix(Tb_)
            ta_ = kin.translationFromHmgMatrix(Ta_)
            tb_ = kin.translationFromHmgMatrix(Tb_)

            err_ = np.linalg.norm(Ra_.dot(tx) + ta_ - Rx.dot(tb_) - tx)      # Ra_ * tx + ta = Rx * tb + tx
            cost_ = lmd[0] * err_ + lmd[1] * constraint
            cost.append(cost_)
        return cost

    Rx0 = kin.rotationFromHmgMatrix(X0)
    tx0 = kin.translationFromHmgMatrix(X0)
    x0 = np.concatenate((quaternion_from_matrix(Rx0), tx0.reshape(3)), axis=0)
    res = scipy.optimize.least_squares(fun=lossfn, x0=x0, method='lm')
    X = kin.homogMatfromQuatAndTrans(res.x[0:4], res.x[4:7])
    toc = time.perf_counter()
    elapsed_time = toc - tic
    return X, elapsed_time

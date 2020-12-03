"""
AX=YB
1. Li 
2. Shah 
3. Tabb, Zc1
4. Tabb, Zc2

"""
import numpy as np
#from utils import kinematics as kin
from . import kinematics as kin
#import kinematics as kin
import time
import scipy.optimize
from pytransform3d.rotations import *


def li(Ta, Tb):
    """li.
    AX = YB 
    Simultaneous robot-world and hand-eye calibration using dual-quaternions and Kronecker product

    :param Ta:
    :param Tb:
    """
    tic = time.perf_counter()
    A = np.empty((1, 24))
    b = np.empty((1, 1))
    for Ta_, Tb_ in zip(Ta, Tb):
        Ra_, ta_ = kin.seperateFromHmgMatrix(Ta_)
        Rb_, tb_ = kin.seperateFromHmgMatrix(Tb_)
        Ra_ = np.transpose(Ra_)
        ta_ = -np.dot(Ra_, ta_)
        A_1 = np.concatenate((np.kron(Ra_, np.eye(3)), np.kron(-np.eye(3), np.transpose(Rb_)), np.zeros((9, 6))), axis=1)
        A_2 = np.concatenate((np.zeros((3, 9)), np.kron(np.eye(3), np.transpose(tb_)), -Ra_, np.eye(3)), axis=1)
        A = np.concatenate((A, A_1, A_2), axis=0)
        b = np.concatenate((b, np.zeros((9, 1)), ta_), axis=0)
    
    x = np.dot(np.linalg.pinv(A[1:]), b[1:])
    X = x[:9].reshape((3, 3))
    u, _, v = np.linalg.svd(X)
    X = np.dot(u, v)
    if np.linalg.det(X) < 0:
        X = np.dot(np.dot(u, np.diag([1, 1, -1])), v)
    X = kin.homogMatfromRotAndTrans(X, x[18:21])

    Y = x[9:18].reshape((3, 3))
    u, _, v = np.linalg.svd(Y)
    Y = np.dot(u, v)
    if np.linalg.det(Y) < 0:
        Y = np.dot(np.dot(u, np.diag([1, 1, -1])), v)
    Y = kin.homogMatfromRotAndTrans(Y, x[21:24])
    toc = time.perf_counter()
    elapsed_time = toc - tic
#    print('Time(Li):', elapsed_time)
    return X, Y, elapsed_time


def shah(Ta, Tb):
    """shah.
    AX = YB 
    Simultaneous Robot/World and Tool/Flange Calibration by Solving Homogeneous Transformation 

    :param Ta:
    :param Tb:
    :return 
    """
    tic = time.perf_counter()
    T = np.zeros((9, 9))

    for Ta_, Tb_ in zip(Ta, Tb):
        Ra_, ta_ = kin.seperateFromHmgMatrix(Ta_)
        Rb_ = kin.rotationFromHmgMatrix(Tb_)
        Ra_ = np.transpose(Ra_)
        ta_ = -np.dot(Ra_, ta_)
        T = T + np.kron(Rb_, Ra_)
        
    u, _, v = np.linalg.svd(T)
    v = np.transpose(v)
    x = v[:, 0]
    y = u[:, 0]
    X = x.reshape((3, 3))

    X = np.sign(np.linalg.det(X)) / np.power(np.abs(np.linalg.det(X)), 1/3) * X
    u, _, v = np.linalg.svd(X)
    X = np.dot(u, v)

    Y = y.reshape((3, 3))
    Y = np.sign(np.linalg.det(Y)) / np.power(np.abs(np.linalg.det(Y)), 1/3) * Y
    u, _, v = np.linalg.svd(Y)
    Y = np.dot(u, v)

    # Finding tx and ty
    A = np.zeros((1, 6))
    b = np.zeros((1, 1))

    for Ta_, Tb_ in zip(Ta, Tb):
        Ra_, ta_ = kin.seperateFromHmgMatrix(Ta_)
        Rb_, tb_ = kin.seperateFromHmgMatrix(Tb_)
        Ra_ = np.transpose(Ra_)
        ta_ = -np.dot(Ra_, ta_)
        A_ = np.concatenate((-Ra_, np.eye(3)), axis=1)
        A = np.concatenate((A, A_), axis=0)
        b_ = ta_ - np.dot(np.kron(np.transpose(tb_), np.eye(3)), Y.reshape((9, 1)))
        b = np.concatenate((b, b_), axis=0)

    t = np.dot(np.linalg.pinv(A[1:]), b[1:])

    X = kin.homogMatfromRotAndTrans(X, t[0:3])
    # !!! I don't know the reason. Why I should traspose the Y !!! #
    Y = kin.homogMatfromRotAndTrans(Y.T, t[3:6])
    toc = time.perf_counter()
    elapsed_time = toc - tic
#    print('Time(Shah):', elapsed_time)
    return X, Y, elapsed_time


def tabb_zc1(Ta, Tb, X0, Y0, cost_fn_tol):
    """tabb_zc1.

    :param Ta:
    :param Tb:
    :param X0:
    :param Y0:

    :return X: Transformation matrix from robot base to world
    :return Y: Transformation matrix from robot hand to eye
    """
    tic = time.perf_counter()

    def lossfn(x):
        cost = []
        X = kin.homogMatfromQuatAndTrans(x[0:4], x[4:7])
        Y = kin.homogMatfromQuatAndTrans(x[7:11], x[11:14])
        for Ta_, Tb_ in zip(Ta, Tb):
            Ta_ = kin.homogeneousInverse(Ta_)
            err = np.dot(Ta_, X) - np.dot(Y, Tb_)
            # ori_e = kin.quaternionfromHmgMatrix(err) # different with Ali's code. cf) norm of quaternion
            ori_e = kin.rotationFromHmgMatrix(err) # Frobenius norm in this code
            pos_e = kin.translationFromHmgMatrix(err)
            cost_ = np.linalg.norm(ori_e) + np.linalg.norm(pos_e)
            cost.append(cost_)
        return cost

    Rx0 = kin.rotationFromHmgMatrix(X0)
    Ry0 = kin.rotationFromHmgMatrix(Y0)
    tx0 = kin.translationFromHmgMatrix(X0)
    ty0 = kin.translationFromHmgMatrix(Y0)
    x0 = np.concatenate((quaternion_from_matrix(Rx0), tx0.reshape(3), quaternion_from_matrix(Ry0), ty0.reshape(3)), axis=0)
    ''' lm method
    - jac: only '2-point'
    - ftol: Tolerance for termination by the change of the cost function. Default is 1e-8.
      xtol: Tolerance for termination by the change of the independent variables. Default is 1e-8. 
      gtol: Tolerance for termination by the norm of the gradient. Default is 1e-8. 
    If None, the termination by this condition is disabled.
    - loss: only 'linear' option, so 'f_scale' also cannot control.
    - max_nfec
    '''
    res = scipy.optimize.least_squares(fun=lossfn, x0=x0, method='lm', ftol=cost_fn_tol)
    X = kin.homogMatfromQuatAndTrans(res.x[0:4], res.x[4:7])
    Y = kin.homogMatfromQuatAndTrans(res.x[7:11], res.x[11:14])
    toc = time.perf_counter()
    elapsed_time = toc - tic
#    print('Time(Tabb_zc1):', round(elapsed_time, 2))
    return X, Y, elapsed_time


def tabb_zc2(Ta, Tb, X0, Y0, cost_fn_tol):
    """tabb_zc2.

    :param Ta:
    :param Tb:
    :param X0:
    :param Y0:

    :return X: Transformation matrix from robot base to world
    :return Y: Transformation matrix from robot hand to eye
    """
    tic = time.perf_counter()
    
    def lossfn(x):
        cost = []
        X = kin.homogMatfromQuatAndTrans(x[0:4], x[4:7])
        Y = kin.homogMatfromQuatAndTrans(x[7:11], x[11:14])
        for Ta_, Tb_ in zip(Ta, Tb):
            Ta_ = kin.homogeneousInverse(Ta_)
            err = Ta_ - np.dot(np.dot(Y, Tb_), kin.homogeneousInverse(X))
            # ori_e = kin.quaternionfromHmgMatrix(err) # different with Ali's code. cf) norm of quaternion
            ori_e = kin.rotationFromHmgMatrix(err) # Frobenius norm in this code
            pos_e = kin.translationFromHmgMatrix(err)
            cost_ = np.linalg.norm(ori_e) + np.linalg.norm(pos_e)
            cost.append(cost_)
        return cost

    Rx0 = kin.rotationFromHmgMatrix(X0)
    Ry0 = kin.rotationFromHmgMatrix(Y0)
    tx0 = kin.translationFromHmgMatrix(X0)
    ty0 = kin.translationFromHmgMatrix(Y0)
    x0 = np.concatenate((quaternion_from_matrix(Rx0), tx0.reshape(3), quaternion_from_matrix(Ry0), ty0.reshape(3)), axis=0)
    ''' lm method
    - jac: only '2-point'
    - ftol: Tolerance for termination by the change of the cost function. Default is 1e-8.
      xtol: Tolerance for termination by the change of the independent variables. Default is 1e-8. 
      gtol: Tolerance for termination by the norm of the gradient. Default is 1e-8. 
    If None, the termination by this condition is disabled.
    - loss: only 'linear' option, so 'f_scale' also cannot control.
    - max_nfec
    '''
    res = scipy.optimize.least_squares(fun=lossfn, x0=x0, method='lm', ftol=cost_fn_tol)
    X = kin.homogMatfromQuatAndTrans(res.x[0:4], res.x[4:7])
    Y = kin.homogMatfromQuatAndTrans(res.x[7:11], res.x[11:14])
    toc = time.perf_counter()
    elapsed_time = toc - tic
#    print('Time(Tabb_zc2):', round(elapsed_time, 2))
    return X, Y, elapsed_time

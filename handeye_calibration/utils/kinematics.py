import numpy as np
from math import *
import cv2
import math
from pytransform3d.rotations import matrix_from_quaternion, quaternion_from_matrix


def homogMatfromRotAndTrans(R, tvec):
    """homogMatfromRotAndTrans.

    :param R:
    :param tvec:
    """
    if tvec.shape == (3,):
        tvec = np.expand_dims(tvec, axis=1)
    T = np.concatenate((R, tvec), axis=1)
    T = np.concatenate((T, np.array([[0, 0, 0, 1]])), axis=0)
    return T

def homogeneousInverse(T):
    R = T[:3, :3]
    t = T[:3, 3]
    t = np.expand_dims(t, axis=1)
    invT = np.concatenate((R.T, np.dot(-R.T, t)), axis=1)
    invT = np.concatenate((invT, np.array([[0, 0, 0, 1]])), axis=0)
    return invT


# def rotate(original_vector, rotation_matrix):
#     """
#     Input: Angle for rotation, bool type(degree or radian)
#     Return: Rotation matrix(type: numpy, shape: 3,3)
#     """
#     rotated_vec = np.dot(rotation_matrix, original_vector)
#     return rotated_vec


def rotateXaxis(ang, is_deg):
    """
    Input: Angle for rotation, bool type(degree or radian)
    Return: Rotation matrix(type: numpy, shape: 3,3)
    """
    if is_deg:
        ang = np.deg2rad(ang)
    rotX = np.array([[1, 0, 0], [0, cos(ang), -sin(ang)], [0, sin(ang), cos(ang)]])
    return rotX

def rotateYaxis(ang, is_deg):
    """
    Input: Angle for rotation, bool type(degree or radian)
    Return: Rotation matrix(type: numpy, shape: 3,3)
    """
    if is_deg:
        ang = np.deg2rad(ang)
    rotY = np.array([[cos(ang), 0, sin(ang)], [0, 1, 0], [-sin(ang), 0, cos(ang)]])

    return rotY

def rotateZaxis(ang, is_deg):
    """
    Input: Angle for rotation, bool type(degree or radian)
    Return: Rotation matrix(type: numpy, shape: 3,3)
    """
    if is_deg:
        ang = np.deg2rad(ang)
    rotZ = np.array([[cos(ang), -sin(ang), 0], [sin(ang), cos(ang), 0], [0, 0, 1]])
    return rotZ

#def rotationMatrix(r, p, y, is_deg, is_homog):
def rotationMatrix(xyz_angle, is_deg, is_homog):
    """
    Generate a rotation matrix with 3 rotations
    -> RotZ * RotY * RotX
    param: [rx, ry, rz] 
    param: bool type(degree or radian)
    param: bool type(3x3 or 4x4)
    return: Rotation matrix(type: numpy, shape: 3,3)
    """
    R1 = rotateZaxis(xyz_angle[2], is_deg)
    R2 = rotateYaxis(xyz_angle[1], is_deg)
    R3 = rotateXaxis(xyz_angle[0], is_deg)
    R = R1.dot(R2).dot(R3)
    if is_homog:
        R = np.hstack((R, np.zeros((3, 1))))
        R = np.vstack((R, np.array([[0, 0, 0, 1]])))
    return R

def translationVector(x, y, z):
    """
    Generate a translation vector
    Input: x, y, z
    Output: Vector(type: numpy, shape: 3,1)
    """
    t_vec = np.array([[x], [y], [z]])
    return t_vec

def translationMatrix(x, y, z):
    """
    Generate a translation matrix
    Input: x, y, z
    Output: Matrix(type: numpy, shape: 4,4)
    """
    t_mat = np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
    return t_mat

def homogeneousMatrix(x, y, z, rx, ry, rz, is_deg):
    """
    Generate a translation matrix
    Input: x, y, z, r, p, y, bool type(degree or radian)
    Output: Matrix(type: numpy, shape: 4,4)
    """
    is_homog = True
    homog_matrix = np.dot(translationMatrix(x, y, z), rotationMatrix(rx, ry, rz, is_deg, is_homog))
    return homog_matrix

def generatePointSet(num, dim, min, max):
    """
    Generate a point set
    :param num: The number of points
    :param dim: The dimension of points(2d or 3d)
    :param min: The minimum of each point value
    :param max: The maximum of each point value
    :return: Point set(Nx2 or Nx3)
    """
    return np.random.uniform(min, max, (num, 3))

def transform(points, transformation_matrix_a2b):
    """
    Transform point from A to B
    :param points: Points in 3d
    :param transformation_matrix_a2b: Homogeneous matrix(4x4)
    :return: Transformed point
    """
    if points.shape[0] > 1:
        transformed = []
        for p in points:
            point = np.append(p, [1])
            transformed.append(np.dot(transformation_matrix_a2b, point))
    else:
        point = np.append(points, [1])
        transformed = np.dot(transformation_matrix_a2b, point)

    # List to Numpy
    transformed = np.array(transformed)
    return transformed[:, 0:3]

def rotationFromHmgMatrix(T):
    R = T[:3, :3]
    return R

def translationFromHmgMatrix(T):
    t = T[:3, 3]
    # Expand
    t = np.expand_dims(t, axis=1)
    return t

def rodVectorToAngle(rvec):
    """
    cv2.Rodrigues function returns rotation vector with angle, so the norm of the vector isn't 1.

    :param rvec:
    :return: angle
    :return: normal_rvec
    """
    angle = np.linalg.norm(rvec)
    normal_rvec = rvec / angle
    return angle, normal_rvec

def rotMatrixToRodVector(R, R_tol):
    # a = (np.trace(R) - 1) / 2
    # if a == -1:
    #     rvec = np.array([sqrt((1 + R[0, 0]) / 2), sqrt((1 + R[1, 1]) / 2), sqrt((1 + R[2, 2]) / 2)])
    # elif a == 1:
    #     rvec = np.zeros(3)
    # else:
    #     to_normal = np.sqrt((3 - np.trace(R)) * (1 + np.trace(R)))
    #     rvec = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]] / to_normal)
    # angle = math.acos((np.trace(R) - 1) / 2)
    # print(np.linalg.inv(R))
    # print(np.transpose(R))
    # print(np.array_equal(np.transpose(R), np.linalg.inv(R)))
    # print(np.sum(np.linalg.inv(R) - np.transpose(R)))
    # print(math.isclose(np.sum(np.linalg.inv(R) - np.transpose(R)), 0.1e-30, rel_tol=0.01))
    # assert (isRotationMatrix(R))
    if np.isclose(np.linalg.det(R), 1):
        a = np.array(np.transpose(R))
        b = np.linalg.inv(R)
        # print('A^-1 - A^T = ', np.sum(a - b))
        if abs(np.sum(a - b)) < R_tol: #1e-10:
            # print('The inverse matrix was same with transpose matrix(Second property).')
            rvec = np.empty((3, 1))
            cv2.Rodrigues(R, rvec)
            angle = np.linalg.norm(rvec)
            if np.isclose(angle, 0):
                print('Angle is almost zero.')
            else:
                norm_rvec = rvec / angle
                return norm_rvec, angle
        else:
            print('This matrix', R, ' is not orthogonal')
            print('R^T =! R^-1', abs(np.sum(a-b)))

    elif np.isclose(np.linalg.det(R), -1):
        print('This matrix', R, ' is improper rotation(roto-reflection).')

    else:
        print('This matrix', R, ' is not a rotation matrix.')
        print('The determinant of R:', np.linalg.det(R))

def angleBtwVectors(a, b):
    cos_ab = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return math.acos(cos_ab)

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R, R_tol):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < R_tol 

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotMatrixToEuler(R, R_tol):
    assert (isRotationMatrix(R, R_tol))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

def orientationErrorByRotation(R1, R2):
    """orientationErrorByRotation.

    :param R1: Numpy array (3, 3)
    :param R2: Numpy array (3, 3) 
    :return Error: Scalar
    """
    rel_R = np.dot(np.linalg.inv(R1), R2)
    ori_error = 0.5 * np.array([rel_R[2, 1] - rel_R[1, 2], rel_R[0, 2] - rel_R[2, 0], rel_R[1, 0] - rel_R[0, 1]] ) 

    return np.linalg.norm(ori_error)

def orientationErrorByRod(R1, R2):
    rvec1 = np.empty((3,1))
    rvec2 = np.empty((3,1))
    cv2.Rodrigues(R1, rvec1)
    cv2.Rodrigues(R2, rvec2)
    ang1, rvec1 = rodVectorToAngle(rvec1)
    ang2, rvec2 = rodVectorToAngle(rvec2)
    diff_ang = np.arccos(np.dot(rvec1.reshape(1, 3), rvec2) / (np.linalg.norm(rvec1) * np.linalg.norm(rvec2)))
    return diff_ang

def homogMatfromQuatAndTrans(q, tvec):
    rot = matrix_from_quaternion(q)
    return homogMatfromRotAndTrans(rot, tvec)

def quaternionfromHmgMatrix(T):
    rot = rotationFromHmgMatrix(T)
    return quaternion_from_matrix(rot) 

def seperateFromHmgMatrix(T):
    R = rotationFromHmgMatrix(T)
    t = translationFromHmgMatrix(T)
    return R, t

def normalizeRotation(R):
    det = np.linalg.det(R)
    R = np.cbrt(np.sign(det) / np.absolute(det)) * R
    w = np.empty((3, 1))
    u = np.empty((3, 3))
    vt = np.empty((3, 3))
    cv2.SVDecomp(R, w, u, vt)
    R = np.dot(u, vt)
    return R


def rot2quatMinimal(R):

    trace = np.trace(R)

    if trace > 0:
        S = np.sqrt(trace + 1) * 2
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    return np.array([qx, qy, qz])

def skew(vec):
    R = np.array([[0, -vec[2], vec[1]], 
        [vec[2], 0, -vec[0]], 
        [-vec[1], vec[0], 0]])
    return R



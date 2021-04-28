import os
import numpy as np
import argparse

parser = argparse.ArgumentParser(description='Compute Fiducial Registration')
parser.add_argument('input_1', metavar='INPUT_FILE', help='1st csv file of extracted fiducial coordinates')
parser.add_argument('input_2', metavar='INPUT_FILE', help='2nd csv file of extracted fiducial coordinates')
args = parser.parse_args()

point_set_1 = np.loadtxt(args.input_1, delimiter=',')
point_set_2 = np.loadtxt(args.input_2, delimiter=',')

# Function borrowed from https://github.com/nghiaho12/rigid_transform_3D
def rigid_transform_3D(A, B):
    '''
    Input: expects 3xN matrix of points
    Returns R,t
    R = 3x3 rotation matrix
    t = 3x1 column vector
    '''
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B

    return R, t

R, t = rigid_transform_3D(point_set_2.T, point_set_1.T)
H = np.concatenate((R,t), axis=1)
H = np.concatenate((H,[[0,0,0,1]]), axis=0)

# Save result to file
# np.savetxt('result.csv', H.flatten()[None], fmt='%.5f')

# Print result in terminal, copy/paste into Meshlab -> Matrix: Set/Copy Transformation
np.set_printoptions(suppress=True)
print(*H.flatten(), sep=' ')
import os
import numpy as np
from utils import load_data
from utils import calibrate
from utils import compute_error

data_foler = 'data/'

methods = ['TSA', 'PAR', 'HOR', 'AND', 'DAN', 'AXC1', 'AXC2', 'HORN', 'OURS', 'LI', 'SHA', 'TZC1', 'TZC2']
exclude_method = [10, 11, 12, 13]
observ_rank = False # Rank of the matrix
observ_pose = False # Use redundant stations
verification = False # Use a verification in other poses
validate_folder = 'data_validate/'
show_plot = True

def validate(A, B, X_est):
    """validate.

    :param A: Numpy array, (total_poses, 4, 4)
    :param B: Numpy array, (total_poses, 4, 4)
    :param X_est: Numpy array, (total_methods, 4, 4)
    :return pos_e, ori_e: Numpy array (1, total_method)
    """

    pos_e = []
    ori_e = []
    for X_est_ in X_est:
        pos_e_, ori_e_ = compute_error.woGroundTruth(A, B, X_est_)
        pos_e.append(pos_e_)
        ori_e.append(ori_e_)
    return pos_e, ori_e


if __name__ == "__main__":

    data_path = os.path.join(os.getcwd() + '/', data_foler)

    if not exclude_method:
        total_method = 13
        ex_method = []
    else:
        total_method = 13 - len(exclude_method)
        ex_method = list(map(int, exclude_method))

    for idx, i in enumerate(ex_method):
        del methods[(i-1)-idx]

    lm_tol = 1e-4
    R_tol = 1e-7

    # Load data
    T_robot = load_data.read_robot_pose_from_dir(data_path)
    T_eye = load_data.read_image_from_dir(data_path)
    num_of_poses = len(T_eye)
    if num_of_poses != len(T_robot):
        print("WARNING: load robot poses not equal to the number of images.")

    # To validate the new pose using the estimated result
    # T_robot_vali = load_data.read_robot_pose_from_dir(validate_folder)
    # T_eye_vali = load_data.read_image_from_dir(validate_folder)

    min_num_pose = 9
    max_num_pose = len(T_eye)
    
    if observ_pose:
        print('Effect of the number of poses')
        pos_e = np.empty((1, total_method))
        ori_e = np.empty((1, total_method))
        for n_pose in range(min_num_pose, max_num_pose + 1):
            print(n_pose, 'poses')
            A = T_robot[:n_pose, :]
            B = T_eye[:n_pose, :]
            X_axxb = calibrate.axxb_method(A, B, lm_tol, ex_method)
            X_axyb, Y_axyb = calibrate.axyb_method(A, B, lm_tol, ex_method)
            X_est = np.concatenate((X_axxb, Y_axyb), axis=0)
            if verification:
                pos_e_, ori_e_ = compute_error.woGroundTruth(T_robot_vali, T_eye_vali, X_est, R_tol)
                pos_e = np.concatenate((pos_e, np.expand_dims(pos_e_, axis=0)), axis=0)
                ori_e = np.concatenate((ori_e, np.expand_dims(ori_e_, axis=0)), axis=0)
            else:
                # TODO
                # 1. T_robot, T_eye : total or calib or vali?
                # 2. Total error = position error + orientation error? no weight? 
                pos_e, ori_e = compute_error.woGroundTruth(T_robot, T_eye, X_est, R_tol)
                err = pos_e + ori_e
                min_idx = np.argmin(err)
                print('X(at the minumum error): ')
                print(X_est[min_idx])
        if verification:
            compute_error.graphPlot(pos_e[1:], ori_e[1:], ex_method, 1)

    elif observ_rank:
        from utils import axxb
        M_rank, N_rank = axxb.Tsai_rank(T_robot, T_eye)
        print("M rank :", M_rank)
        print("N rank :", N_rank)
        
    else:
        X_axxb = calibrate.axxb_method(T_robot, T_eye, lm_tol, ex_method)
        X_axyb, Y_axyb = calibrate.axyb_method(T_robot, T_eye, lm_tol, ex_method)
        X_est = np.concatenate((X_axxb, Y_axyb), axis=0)
        pos_e, ori_e = compute_error.woGroundTruth(T_robot, T_eye, X_est, R_tol)
        err = pos_e + ori_e
        min_idx = np.argmin(err)
        np.set_printoptions(suppress=True)
        print('X(at the minumum error):')
        print(X_est[min_idx])
        
        if show_plot:
            import matplotlib.pyplot as plt
            from pytransform3d.plot_utils import make_3d_axis
            from pytransform3d.transformations import random_transform
            from pytransform3d.transform_manager import TransformManager
            
            T_hand2eye = X_est[min_idx]
            tm = TransformManager()
            # add_transform(child_frame, parent_frame, H):
            tm.add_transform("cam", "ee", T_hand2eye)
            plt.figure(figsize=(5, 5))
            ax = make_3d_axis(100)
            ax = tm.plot_frames_in("ee", ax=ax, s=50)
            ax.view_init()
            plt.show()

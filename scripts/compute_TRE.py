import numpy as np
import os

folder = "1932f"
file_gt = "skull_CT_postop_targets.csv"
file_exp = "2nd_scan_targets.csv"
data_gt = np.genfromtxt(os.path.join(folder, file_gt), delimiter=',')
data_exp = np.genfromtxt(os.path.join(folder, file_exp), delimiter=',')

TREs = []
for point_gt, point_exp in zip(data_gt, data_exp):
    diff = point_gt - point_exp
    diff = np.linalg.norm(diff)
    TREs.append(diff)

TREs = np.array(TREs).reshape((-1,1))
file_tre = folder + "_" + file_gt.split("_")[1] + "_" + file_exp.split("_")[1] + ".csv"
np.savetxt(file_tre, TREs, delimiter=',')
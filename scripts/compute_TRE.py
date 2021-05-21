import numpy as np
import os

folder = "data"
file_gt = "skull_CT_postop_with_targets_registered_targets.csv"
file_exp = "XXXXXXXXX-XXXXXXXX.csv"
data_gt = np.genfromtxt(os.path.join(folder, file_gt), delimiter=',')
data_exp = np.genfromtxt(os.path.join(folder, file_exp), delimiter=',')

TREs = []
for point_gt, point_exp in zip(data_gt, data_exp):
    diff = point_gt - point_exp
    diff = np.linalg.norm(diff)
    TREs.append(diff)

TREs = np.array(TREs).reshape((-1,1))
file_tre = "TRE_" + file_gt.split(".")[0] + "_to_" + file_exp.split(".")[0] + ".csv"
np.savetxt(file_tre, TREs, delimiter=',')
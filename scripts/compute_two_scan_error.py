import pyvista as pv
import numpy as np

file_gt = "../analysis/skull_3/skull_3_defect_3_wall_segmented.stl"
file_exp = "../analysis/skull_3/skull_3_defect_3_with_drape_markers_registered_defect_wall_segmented.ply"
data_gt = pv.read(file_gt)
data_exp = pv.read(file_exp)

print("==============")
print("skull_3")
print("gt:")
print(data_gt.center)
print("exp:")
print(data_exp.center)
diff = np.array(data_gt.center) - np.array(data_exp.center)
print("diff")
print(diff)
print(np.linalg.norm(diff))

file_gt = "../analysis/skull_5/skull_5_defect_3_wall_segmented.stl"
file_exp = "../analysis/skull_5/skull_5_defect_3_with_drape_markers_registered_defect_wall_segmented.ply"
data_gt = pv.read(file_gt)
data_exp = pv.read(file_exp)

print("==============")
print("skull_5")
print("gt:")
print(data_gt.center)
print("exp:")
print(data_exp.center)
diff = np.array(data_gt.center) - np.array(data_exp.center)
print("diff")
print(diff)
print(np.linalg.norm(diff))
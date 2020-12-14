import pyvista as pv
import numpy as np

import os

# This file is a pre-processor to take a CT mesh model then remove the inner layer of the mesh
# for registration from CT to 3d scan of the patien's head

self_dir = os.path.dirname(os.path.realpath(__file__))
data_dir = self_dir + "/../Data/"
input_file = "skull_5.stl"
output_file = "skull_5_outer_surface.stl"

# load CT mesh
mesh_ct = pv.read(data_dir + input_file)

# construct vectors from center to each cell centers, then perform element-wise dot product
removals = []
vectors_center_to_cells = mesh_ct.cell_centers().points - mesh_ct.center
for vec, cell_normal in zip(mesh_ct.cell_normals, vectors_center_to_cells):
    removals.append(np.dot(vec, cell_normal))
removals = np.array(removals)

# If computed result is negative, the cell is facing inward, therefore remove these cells and its associated points
mesh_ct.remove_cells(removals < 0)

# clean the mesh
mesh_ct.extract_largest(inplace=True)

# output the mesh
mesh_ct.save(data_dir + output_file, binary=True)
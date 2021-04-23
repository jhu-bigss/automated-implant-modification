import pyvista as pv
import trimesh
import numpy as np
from scipy.spatial import cKDTree

import os
import time
import argparse

# This file is a pre-processor to extract spherical fiducials on a head model
# for registration from 2nd to 3rd scan of the patien's head
# Usage:
#       python extract_fiducial_centers.py input_mesh_file_path

# parser = argparse.ArgumentParser(description='Extract spherical fiducials')
# parser.add_argument('mesh_file', metavar='INPUT_FILE', help='Input mesh file path')
# args = parser.parse_args()

# input_file = args.mesh_file
# data_dir = os.path.dirname(input_file)
# input_file_name = os.path.basename(input_file)
# output_file = input_file_name.split('.')[0] + "_outer_surface.stl"


# load CT mesh
input_file = "../Spherical_fiducials_segmentation/TEST_30mm.ply" # change input file
RADIUS = 15
mesh_head = pv.read(input_file)
tree = cKDTree(mesh_head.points)
# trimesh_head = trimesh.load(input_file)

plotter = pv.Plotter()
plotter.add_mesh(mesh_head, name="head")

# define point picking callback
def point_picking_callback(mesh, point_id):
    # For each fiducial:
    # 1. manual select an arbitrary point on it
    # 2. get cluster around the point () 
    # 3. fit sphere on the cluster, get center of the fitted sphere
    # 4. create a sphere given radius, run ICP between the created sphere and the cluster, initialized at the center from prev. step
    picked_point = mesh.points[point_id]
    print("fiducial selected: " + str(picked_point))

    # get neighboring points within a radius 
    ball_neighbors = mesh.points[tree.query_ball_point(picked_point, 25)] # 25 can be tuned here
    # debug
    plotter.add_points(ball_neighbors, color='w', point_size=5, name="S")

    time.sleep(2)
    # fit sphere
    center, radius, error = trimesh.nsphere.fit_nsphere(ball_neighbors)
    # # debug
    # print(center)
    # print(radius)
    fitted_sphere = pv.Sphere(radius, center)
    plotter.add_mesh(fitted_sphere, color='w', name="S")

    time.sleep(2)
    # create a sphere given raidius and center
    fitted_sphere = pv.Sphere(RADIUS, center)
    plotter.add_mesh(fitted_sphere, color='w', name="S")

    time.sleep(2)
    # run ICP
    transformation_matrix, transformed, cost = trimesh.registration.icp(ball_neighbors, fitted_sphere.points)
    fitted_sphere.transform(transformation_matrix, inplace=True)
    # plotter.add_mesh(fitted_sphere, color='w', name="S")

plotter.enable_point_picking(callback=point_picking_callback, show_message=False, point_size=10, color='red', use_mesh=True, show_point=True)
plotter.show()
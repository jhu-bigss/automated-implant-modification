import pyvista as pv
import trimesh
import numpy as np
from scipy.spatial import cKDTree

import os
import time
import argparse

SLEEP_TIME = 1
# This file is a pre-processor to extract spherical fiducials on a head model
# for registration from 2nd to 3rd scan of the patien's head
# Usage:
#       python extract_fiducial_centers.py input_mesh_file_path radius (optional: -d True/False)
# During the GUI:
#       press "s" key to save the center of fiducials

parser = argparse.ArgumentParser(description='Extract spherical fiducials')
parser.add_argument('mesh_file', metavar='INPUT_FILE', help='Input mesh file path')
parser.add_argument('radius', type=float, help='Radius of the spherical fiducials')
parser.add_argument('-d', "--debug", type=bool, default=False, help='In debug mode or not, default: NOT in debug mode')
args = parser.parse_args()

if not args.mesh_file or not args.radius:
    print("Missing input file or radius of sphere")
    exit(1)

# params
input_file = args.mesh_file
data_dir = os.path.dirname(input_file)
input_file_name = os.path.basename(input_file)
output_file = os.path.splitext(input_file_name)[0] + ".csv"
radius = args.radius
neighbor_search_radius = radius * 1.6 # the factor can be tuned, the user is supposed to pick a point on the top of the sphere
debug = args.debug
sphere_color = "r" # "w"


# load CT mesh
# input_file = "../Spherical_fiducials_segmentation/TEST_30mm.ply" # change input file
# input_file = "../skull_3/skull_3_defect_3_with_head_markers.stl" # change input file
# input_file_name = os.path.basename(input_file)
# output_file = os.path.splitext(input_file_name)[0] + ".csv"

mesh_head = pv.read(input_file)
tree = cKDTree(mesh_head.points)

plotter = pv.Plotter()
plotter.add_mesh(mesh_head, rgb=True, name="head")
poly = pv.PolyData() # center of fiducials

# define point picking callback
def point_picking_callback(mesh, point_id):
    # For each fiducial:
    # 1. manual select an arbitrary point on it
    # 2. get cluster around the point () 
    # 3. fit sphere on the cluster, get center of the fitted sphere
    # 4. create a sphere given radius, run ICP between the created sphere and the cluster, initialized at the center from prev. step

    # # delete selected point if existing:
    # if mesh is not None and mesh is not mesh_head:
    #     actor_collection = plotter.renderer.GetActors()
    #     for i in range(actor_collection.GetNumberOfItems()):
    #         actor = actor_collection.GetNextActor()
    #         if actor == mesh:

    if mesh is mesh_head:

        print("fiducial selected")
        picked_point = mesh.points[point_id]
        sphere_name = "S" + str(poly.n_points)

        if poly.n_points == 0:
            poly.points = picked_point
        else:
            poly.points = np.vstack((poly.points, picked_point))

        # add labels for picked point
        poly["labels"] = [f"Label {i}" for i in range(poly.n_points)]
        plotter.add_point_labels(poly, "labels", point_size=20, font_size=40)

        # get neighboring points within a radius 
        ball_neighbors = mesh.points[tree.query_ball_point(picked_point, neighbor_search_radius)]
        plotter.add_points(ball_neighbors, color=sphere_color, point_size=5, name=sphere_name)

        # fit sphere
        center, radius_fitted, error = trimesh.nsphere.fit_nsphere(ball_neighbors)
        fitted_sphere = pv.Sphere(radius_fitted, center)
        if debug:
            time.sleep(SLEEP_TIME)
        plotter.add_mesh(fitted_sphere, color=sphere_color, name=sphere_name)

        # create a sphere given raidius and center
        fitted_sphere = pv.Sphere(radius, center)
        if debug:
            time.sleep(SLEEP_TIME)
        plotter.add_mesh(fitted_sphere, color=sphere_color, name=sphere_name)

        # run ICP
        transformation_matrix, transformed, cost = trimesh.registration.icp(ball_neighbors, fitted_sphere.points) # "transformed" are points used for registration
        transformation_matrix = np.linalg.inv(transformation_matrix)
        if debug:
            time.sleep(SLEEP_TIME)
        fitted_sphere.transform(transformation_matrix, inplace=True)
        # update poly points
        print(fitted_sphere.center)
        poly.points[-1] = fitted_sphere.center
    
# define key callback to save centers: key "s"
def key_callback():
    if poly.n_points > 0:
        # save center of fiducials
        np.savetxt(output_file, poly.points, fmt='%.5f', delimiter=",")
        print("csv file saved")
    else:
        print("No points to save")

plotter.enable_point_picking(callback=point_picking_callback, show_message=False, point_size=10, color='black', use_mesh=True, show_point=True)
plotter.add_key_event("s", key_callback)
plotter.show()
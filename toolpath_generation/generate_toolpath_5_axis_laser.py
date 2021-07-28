import pyvista as pv
import numpy as np
from skspatial.objects import Line, Points, Plane

import argparse

parser = argparse.ArgumentParser(description='5-Axis Toolpath Generation')
parser.add_argument('mesh_cci', metavar='INPUT_FILE', help='Input CCI mesh file path')
parser.add_argument('mesh_defect', metavar='INPUT_FILE', help='Input defect edge mesh file path')
parser.add_argument('transform_reg', metavar='INPUT_FILE', help='Input transformation file path')
args = parser.parse_args()

# Read data from input arguments
mesh_cci = args.mesh_cci
mesh_defect = args.mesh_defect
tansform_input = args.transform_reg

mesh_implant = pv.read(mesh_cci)
mesh_defect_wall = pv.read(mesh_defect)
transform_mat = np.genfromtxt(tansform_input)

# Transform the CCI and defect edge to the laser space
mesh_implant.transform(transform_mat, inplace=True)
mesh_defect_wall.transform(transform_mat, inplace=True)

# Plotting Themes
pv.global_theme.show_scalar_bar = False

DEBUG = False

if DEBUG:
    from pyvistaqt import BackgroundPlotter
    plotter = BackgroundPlotter()
else:
    plotter = pv.Plotter()

plotter.add_axes()
plotter.add_axes_at_origin(labels_off=True)
plotter.add_mesh(mesh_implant, color='gray')
plotter.add_mesh(mesh_defect_wall)

# project vertices to plane (deprecated)
# plane, center, normal = pv.fit_plane_to_points(mesh_defect_wall.points, return_meta=True)
# plane_pv = pv.Plane(center=center, direction=normal, i_size=100, j_size=100)

normal = np.array([0., 0., 1.])
center = np.array([0., 0., 0.])

# generate cutting contour by the intersection of implant and fitted lines on defect wall
num = 360
angles_total = np.linspace(np.pi/2, 5/2*np.pi, num, endpoint=False) # Start from Y+
distance_threshold = 2
radius = 1
line_length = 10
line_fitting = False

tcp_points = []
tcp_vectors = []

for angle in angles_total:

    # 1. construct a plane (plane_2) by normal and the vector (from polar angle)
    vec_a_in_plane_2 = np.array([radius * np.cos(angle), radius * np.sin(angle), 0])
    normal_2 = np.cross(normal, vec_a_in_plane_2)
    plane_2 = pv.Plane(mesh_defect_wall.center, normal_2, 200, 200, 1, 1) # NOTE: the plane has to be large enough for correct implicit distance
    mesh_defect_wall.compute_implicit_distance(plane_2, inplace=True) # Signed distance
    distances_vertex_to_plane = np.absolute(mesh_defect_wall['implicit_distance'])
    vertices_candidate = mesh_defect_wall.points[np.where(distances_vertex_to_plane < distance_threshold)]

    # 2. filter out incorrect candidates
    vectors_center_to_candidate = vertices_candidate - mesh_defect_wall.center
    dot_product = np.matmul(vectors_center_to_candidate, vec_a_in_plane_2.reshape((3,1)))
    vertices_candidate = vertices_candidate[np.where(dot_product > 0)[0]]

    # 3. fit a line to vertice candidate
    pts_sk = Points(vertices_candidate)

    if line_fitting:
        line_k = Line.best_fit(pts_sk)
        point_k = line_k.point

        # 3.5.1 project the best_fit line to the plane_2, get its unit direction vector
        proj_k_to_normal_2 = line_k.direction.dot(normal_2) * normal_2
        vec_u_in_plane_2 = line_k.direction - proj_k_to_normal_2
        vec_u_in_plane_2 = vec_u_in_plane_2 / np.linalg.norm(vec_u_in_plane_2)
        vec_u = vec_u_in_plane_2
    else:
        plane_X = Plane.best_fit(pts_sk)
        point_k = plane_X.point

        # 3.5.2 the normal vec of the best_fit plane cross product the normal vec of the plane_2
        vec_n_of_plane_x = plane_X.normal.round(3)
        vec_u = np.cross(vec_n_of_plane_x, normal_2)
        # Make sure all the direction are consistant
        if vec_u.dot(normal) <0:
            vec_u = -vec_u

    # 4. ray-tracing for intersection
    intersection_pt, ind = mesh_implant.ray_trace(point_k, point_k + vec_u * line_length)
    if intersection_pt.any():
        tcp_points.append(intersection_pt[0])
        tcp_vectors.append(vec_u)
        # Plotting
        plotter.add_points(intersection_pt, color='red')
        plotter.add_lines(np.array([intersection_pt[0], intersection_pt[0] + vec_u * line_length]), color="green")
    
toolpath = np.column_stack((tcp_points,tcp_vectors))
np.savetxt("cldata.csv", toolpath)

plotter.show()
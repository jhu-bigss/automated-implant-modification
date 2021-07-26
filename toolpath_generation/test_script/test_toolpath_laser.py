import pyvista as pv
import numpy as np
from trimesh import points
from skspatial.objects import Line
from skspatial.objects import Points


def cartesian_to_polar(points):
    """ Function to transform points in 2D cartesian to 2D polar coordinate with height.
    Args:
        points: 2D points with height in cartesian coordinate system, array of (n,3) 
    Returns:
        points_polarized: points in 2D polar with height
    """
    # transform 2D points to polar system (theta, r)
    x, y, h = points[:,0], points[:,1], points[:,2]
    radius = np.sqrt(x**2 + y**2).reshape((-1,1)) # shape: nx1
    angle = np.arctan2(y,x).reshape((-1,1)) # shape: nx1
    points_polarized = np.column_stack((angle, radius, h))

    return points_polarized

def polar_to_cartesian(points_polar, center, axes):
    """ Function to transform points in 2D polar coordinate to 3D cartesian.
    Args:
        points_polar: 2D points in polar coordinate system, array of (n,2) 
        center: 3D coordinate of the origin of 2D polar coordinate system
        axes: x, y, z axis of local coordinate system, represented in 3D, column array of (3, 3)
        onplane: if True, points are projected on the plane
    Returns:
        points_transformed_3d: points in 3D cartesian
    """
    # convert desired_points back to original coordinate system
    angle = points_polar[:,0] # shape: (n,)
    radius = points_polar[:,1] # shape: (n,)
    h = points_polar[:,2]
    x_2d = radius * np.cos(angle)
    y_2d = radius * np.sin(angle)
    points_local = np.column_stack((x_2d, y_2d, h))
    points_3d = center + np.matmul(points_local, axes.T)
    
    return points_3d

plotter = pv.Plotter()

fname_implant = "../Toolpath-Laser/implant-21-1932f_top_surf.ply"
fname_defect_wall = "../Toolpath-Laser/defect_edge.ply"

mesh_implant = pv.read(fname_implant)
mesh_defect_wall = pv.read(fname_defect_wall)

# # used to get fake input
# plane, center, normal = pv.fit_plane_to_points(mesh_defect_wall.points, return_meta=True)
# plane_pv = pv.Plane(center=center, direction=normal, i_size=100, j_size=100)

plotter.add_axes_at_origin()
plotter.add_mesh(mesh_implant)
plotter.add_mesh(mesh_defect_wall)

# Laser space
transformation_input = np.eye(4) # input transformation that represents implant mesh in laser space
normal_temp = [0.8588793, -0.38656333, 0.33599856]
transformation_input[:3,3] = -(mesh_defect_wall.center + np.array(normal_temp)*-10)
mesh_implant.transform(transformation_input)
mesh_defect_wall.transform(transformation_input)

# project vertices to plane
plane, center, normal = pv.fit_plane_to_points(mesh_defect_wall.points, return_meta=True)
plane_pv = pv.Plane(center=center, direction=normal, i_size=100, j_size=100)

# generate cutting contour by the intersection of implant and fitted lines on defect wall
num = 200
angles_uniform = np.linspace(-np.pi, np.pi, num, endpoint=False)
distance_threshold = 1
radius = 1
line_length = 20

for angle in angles_uniform:

    # 1. construct a plane (plane_2) by normal of projection_plane and the vector (from polar angle)
    vector = np.array([radius * np.cos(angle), radius * np.sin(angle), 0])
    normal_2 = np.cross(normal, vector)
    plane_2 = pv.Plane(mesh_defect_wall.center, normal_2, 100, 100, 1, 1) # NOTE: the plane has to be large enough for correct implicit distance
    mesh_defect_wall.compute_implicit_distance(plane_2, inplace=True) # Signed distance
    distances_vertex_to_plane = np.absolute(mesh_defect_wall['implicit_distance'])
    vertices_candidate = mesh_defect_wall.points[np.where(distances_vertex_to_plane < distance_threshold)]

    # 2. filter out incorrect candidates
    vectors_center_to_candidate = vertices_candidate - mesh_defect_wall.center
    dot_product = np.matmul(vectors_center_to_candidate, vector.reshape((3,1)))
    vertices_candidate = vertices_candidate[np.where(dot_product > 0)[0]]

    # 3. fit a line to vertice candidate
    points_sk = Points(vertices_candidate)
    line_fit = Line.best_fit(points_sk)
    start_point = line_fit.point + line_fit.direction*line_length//2
    stop_point = line_fit.point + line_fit.direction*-line_length//2

    plotter.add_lines(np.array([start_point, stop_point]), color="green")

    # 4. ray-tracing for intersection
    intersection_pt, ind = mesh_implant.ray_trace(start_point, stop_point)
    plotter.add_points(intersection_pt, color='red')

plotter.show()
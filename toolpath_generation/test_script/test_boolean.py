import pyvista as pv
import numpy as np

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    # reference: https://stackoverflow.com/questions/45142959/calculate-rotation-matrix-to-align-two-vectors-in-3d-space
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

fname_implant = "Toolpath-Cutting-01/skull_3_implant.stl"
fname_latch = "Toolpath-Cutting-01/latch-mechanism/latch-insert-th-12mm.stl"
fname_contour = "Toolpath-Cutting-01/contour.csv"
fname_defect_wall = "Toolpath-Cutting-01/skull_3_defect_edge_segmented.stl"

mesh_implant = pv.read(fname_implant)
mesh_latch = pv.read(fname_latch) # 12 points, vector to align: (1, 0, 0), (0, 0, 1)
mesh_defect_wall = pv.read(fname_defect_wall)
# print(mesh_latch.points)
spline_curve_fit = pv.PolyData(np.genfromtxt(fname_contour)[:,:3])

plotter = pv.Plotter()

num_latch = 3
latch_reference_point = np.array([7,6,8]) # debug
plane_fit_curve ,_ ,normal_plane_fit_curve = pv.fit_plane_to_points(spline_curve_fit.points, return_meta=True)
distance_threshold = 1

for i in range(num_latch):

    # place latch based on defet contour and defect wall
    # 1. equally distributed 3 latches (by point in spline curve)
    # 2. align latch with tangent vector of spline curve and normal vector of defect wall:
    #   a. (0,0,-1) in latch coord aligns with normal vector of defect wall
    #   b. (-1,0,0) in latch coord aligns with tangent vector of spline curve
    current_latch = mesh_latch.copy()
    spline_curve_point_id = i*len(spline_curve_fit.points)//3 
    next_spline_curve_point_id = spline_curve_point_id + 1
    target_position = spline_curve_fit.points[spline_curve_point_id]
    tangent_vector = spline_curve_fit.points[next_spline_curve_point_id] - target_position

    # debug: visualize tangent vector
    plotter.add_arrows(target_position, tangent_vector, mag=5, color='white')

    vector =  spline_curve_fit.center - target_position
    normal_plane_slice_wall = np.cross(normal_plane_fit_curve, vector)
    plane_slice_wall = pv.Plane(target_position, normal_plane_slice_wall, 100, 100, 1, 1) # NOTE: the plane has to be large enough for correct implicit distance
    mesh_defect_wall.compute_implicit_distance(plane_slice_wall, inplace=True) # Signed distance
    distances_vertex_to_plane = np.absolute(mesh_defect_wall['implicit_distance'])
    vertices_candidate = mesh_defect_wall.points[np.where(distances_vertex_to_plane < distance_threshold)]

    # filter out incorrect candidates
    vectors_center_to_candidate = vertices_candidate - spline_curve_fit.center
    dot_product = np.matmul(vectors_center_to_candidate, vector.reshape((3,1)))
    vertices_candidate = vertices_candidate[np.where(dot_product > 0)[0]]

    # Handle exception where too few candidates were extracted, record the vector id and interpolate for the vector in the next step 
    if len(vertices_candidate) < 3:
        print("Less than 3 points found on defect wall when placing latch " + str(i))
        continue

    # fit plane to candidate vertices and compute vector tool
    plane_fit_wall, _, normal_plane_fit_wall = pv.fit_plane_to_points(vertices_candidate, return_meta=True)
    # check normal direction
    if np.dot(normal_plane_fit_wall, vector) < 0:
        normal_plane_fit_wall = -normal_plane_fit_wall
    plotter.add_arrows(target_position, normal_plane_fit_wall, mag=5, color='green')

    # translate first
    transformation = np.eye(4)
    translation = target_position - latch_reference_point
    transformation[:3, 3] = translation
    current_latch.transform(transformation)

    # two successive rotations around target point
    source_vector_1 = np.array([0,0,-1])
    rotation_matrix = rotation_matrix_from_vectors(source_vector_1, normal_plane_fit_wall)
    current_latch.points = (rotation_matrix @ (current_latch.points - target_position).T).T + target_position
    source_vector_2 = rotation_matrix @ np.array([-1,0,0]).reshape((3,1))
    rotation_matrix = rotation_matrix_from_vectors(source_vector_2, tangent_vector)
    current_latch.points = (rotation_matrix @ (current_latch.points - target_position).T).T + target_position

    plotter.add_mesh(current_latch, opacity=0.5, show_edges=True)

    # get edges of latch by boolean cut and extracting boundary
    cut = mesh_implant - current_latch
    edges = cut.extract_feature_edges(boundary_edges=True, feature_edges=False, manifold_edges=False)
    # keep the edge on the top surface only
    vector = np.cross(tangent_vector, normal_plane_fit_wall)
    plotter.add_arrows(target_position, vector, mag=5, color='blue')
    # edges.slice(normal=vector, origin=target_position-vector*2)
    bodies = edges.split_bodies(label=True)
    print(bodies)
    for i, body in enumerate(bodies):
        if i == 0:
            plotter.add_points(body.points, color="red")
        else:
            plotter.add_points(body.points, color="green")

# diff = mesh_implant.boolean_difference(mesh_latch) 
plotter.add_lines(spline_curve_fit.points, color='red', name='contour')

plotter.add_mesh(mesh_implant, opacity=0.3, show_edges=True)
# plotter.add_points(latch_reference_point, color="blue")
# plotter.add_mesh(mesh_latch, opacity=0.5, show_edges=True)
# plotter.add_axes_at_origin()
plotter.show()
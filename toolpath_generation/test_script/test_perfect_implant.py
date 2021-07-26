import pyvista as pv
import numpy as np

plotter = pv.Plotter()

fname_implant = "Toolpath-Cutting-01/resized_implant.ply"
mesh_implant = pv.read(fname_implant)

# Laser space
plotter.add_axes_at_origin()
transformation_input = np.eye(4) # input transformation that represents implant mesh in laser space
normal_temp = [-0.6436205, -0.7114715, -0.2820656]
transformation_input[:3,3] = -(mesh_implant.center + np.array(normal_temp)*8)
# plotter.add_points(center + normal*8, color="green")
mesh_implant.transform(transformation_input)

edges = mesh_implant.extract_feature_edges(boundary_edges=False, feature_edges=True, manifold_edges=False)

plane, center, normal = pv.fit_plane_to_points(edges.points, return_meta=True)
print(normal)
plane_pv = pv.Plane(center=center, direction=normal, i_size=100, j_size=100)
edges.compute_implicit_distance(plane, inplace=True) # signed distance
vertices_candidate = edges.points[np.where(edges['implicit_distance'] > 0)] # find a way to determine the sign

plotter.add_mesh(mesh_implant, opacity=0.3, show_edges=True)
plotter.add_points(vertices_candidate, color="red")
plotter.add_mesh(plane_pv)

# generate cutting vector
vectors_tool = []
invalid_vector_ids = []
vectors = self.spline_curve_fit.points - self.spline_curve_fit.center
distance_threshold = self.doubleSpinBox_distance_threshold.value()
for vector_id, point, vector in zip(np.arange(len(vectors)), self.spline_curve_fit.points, vectors):

    # for tangent vector
    next_point_id = vector_id + 1
    if next_point_id == len(vectors):
        next_point_id = 1 # first point and last point are the same
    tangent = self.spline_curve_fit.points[next_point_id] - point

    normal_2 = np.cross(self.projection_plane.plane_normal, vector)
    plane_2 = pv.Plane(point, normal_2, 100, 100, 1, 1) # NOTE: the plane has to be large enough for correct implicit distance
    self.mesh_defect_wall.compute_implicit_distance(plane_2, inplace=True) # Signed distance
    distances_vertex_to_plane = np.absolute(self.mesh_defect_wall['implicit_distance'])
    vertices_candidate = self.mesh_defect_wall.points[np.where(distances_vertex_to_plane < distance_threshold)]
        
    # filter out incorrect candidates
    vectors_center_to_candidate = vertices_candidate - self.spline_curve_fit.center
    dot_product = np.matmul(vectors_center_to_candidate, vector.reshape((3,1)))
    vertices_candidate = vertices_candidate[np.where(dot_product > 0)[0]]

    # Handle exception where too few candidates were extracted, record the vector id and interpolate for the vector in the next step 
    if len(vertices_candidate) < 3:
        invalid_vector_ids.append(vector_id)
        vectors_tool.append([0,0,1])
        continue

    # visualization for candidate vertices on the defect wall to fit a plane 
    self.plotter.add_points(point, style='points', color='Green', point_size=20.0, name='point')
    self.plotter.add_points(vertices_candidate, style='points', color='Black', point_size=20.0, name='candidate')

    # fit plane to candidate vertices and compute vector tool
    plane_3, center_3, normal_3 = pv.fit_plane_to_points(vertices_candidate, return_meta=True)
    # check normal_3 direction
    if np.dot(normal_3, vector) < 0:
        normal_3 = -normal_3
    # vector_tool = np.cross(normal_2, normal_3)
    vector_tool = np.cross(tangent, normal_3) # for tangent vector
    vectors_tool.append(vector_tool)

self.plotter.remove_actor('point')
self.plotter.remove_actor('candidate')

vectors_tool = np.array(vectors_tool)
lengths_vectors_tool = np.linalg.norm(vectors_tool, axis=1).reshape((-1,1))
lengths_vectors_tool = np.repeat(lengths_vectors_tool, 3, axis=1)
vectors_tool = vectors_tool / lengths_vectors_tool

threshold_similarity = 0.95
iters = 0
while len(invalid_vector_ids) > 0 or iters == 0:
    print("iters: " + str(iters))
    if len(invalid_vector_ids) > 0:
        print("invalid vectors found")
        print(invalid_vector_ids)
        # find the neighboring and valid vectors, interpolate for invalid vectors_tool
        valid_vector_ids = np.setdiff1d(np.arange(len(vectors_tool)), invalid_vector_ids)
        insertion_vector_ids = np.searchsorted(valid_vector_ids, invalid_vector_ids) # Binary search to find the insertion position
        for invalid_id, insertion_id in zip(invalid_vector_ids, insertion_vector_ids):
            if insertion_id == len(valid_vector_ids):
                next_id = valid_vector_ids[0]
            else:
                next_id = valid_vector_ids[insertion_id]
            prev_id = valid_vector_ids[insertion_id - 1]
            vector_interpolated = (invalid_id - prev_id) * vectors_tool[prev_id] + (next_id - invalid_id) * vectors_tool[next_id]
            vector_interpolated = vector_interpolated / (next_id - prev_id)
            # update vector
            vectors_tool[invalid_id] = vector_interpolated
    
    # Find vectors with large change
    invalid_vector_ids = []
    for i, vector_tool in enumerate(vectors_tool):
        if i == len(vectors_tool) - 1:
            next_vector  = vectors_tool[0]
        else:
            next_vector = vectors_tool[i+1]
        similarity = np.dot(vector_tool, next_vector)
        if similarity < threshold_similarity:
            invalid_vector_ids.append(i)
    iters += 1
    # self.plotter.add_arrows(self.spline_curve_fit.points[invalid_vector_ids], vectors_tool[invalid_vector_ids], mag=3, color='Yellow', name='Weird')

plotter.show()

# bodies = edges.split_bodies(label=True)
# for body in bodies:
#     # if i == 0:
#     #     plotter.add_points(body.points, color="red")
#     # else:
#     #     plotter.add_points(body.points, color="green")
# con = edges.connectivity()
# print(con.array_names)
# print(con.get_array('RegionId'))
# slc = mesh_implant.slice(normal=normal, origin=center)
# slc = edges.slice(normal=normal, origin=center)
# print(slc.array_names)
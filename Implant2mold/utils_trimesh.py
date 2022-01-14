import numpy as np
import cmath
import pyvista 
import pymeshlab

"""
utility functions including transformation from pyvista mesh to Trimesh mesh or inverse;
mesh_cut based on Trimesh

"""

def pyvistaToTrimeshFaces(cells):
        faces = []
        idx = 0
        while idx < len(cells):
            curr_cell_count = cells[idx]
            curr_faces = cells[idx+1:idx+curr_cell_count+1]
            faces.append(curr_faces)
            idx += curr_cell_count+1
        return np.array(faces)

def remove_bounding_points(mold, mode = 'top'):
    """
    Remove the 4 top or bottom end points of the box to generate the final bottom
    or top mold.

    Parameters
    ----------
    mold: Pyvista mesh. Mold after Trimesh cut.
    mode: 'top' or 'bottom'. End points to be removed 

    Return
    -------
    mold: Pyvista mesh
    """
    max_z = mold.bounds[-1]
    points = mold.points
    idx = np.argsort(np.asarray(points), axis = 0)[:,-1]
    idx = idx[::-1] if mode == 'top' else idx
    mold, idxes = mold.remove_points(idx[:4])
    return mold

def split_surf(mesh, mode):
    """
    Find the bottom or top surface of the implant by using normals.

    """
    normals = np.asarray(mesh.point_normals)
    mask = normals[:,2] > 0.1 if mode == 'bottom' else normals[:,2] < -0.1
    idxes = np.arange(len(normals))[~mask]
    splited_surf,_ = mesh.remove_points(idxes)
    
    return splited_surf

def Cartesian2Polar(coords):
    """
    Transfer a sequence of Cartesian coordinates into polar coordinates
    """
    polar_coords = []
    for coord in coords:
        coord_xy = coord[:-1]
        cn = complex(coord_xy[0], coord_xy[1])
        r,phi = cmath.polar(cn) 
        polar_coords.append([r, phi])
    return np.asarray(polar_coords)


def generate_cut_surf(points, l = 5):
    """
    Generate the cut surface based on the edge points

    Parameters
    ----------
    points: edge points
    l:    times of lengthening the ray

    Return
    -------
    mesh: cut surface
    """
    points = np.asarray(points)
    polar_coords = Cartesian2Polar(points) # reorder the points with phase angle
    phi_index= np.argsort(polar_coords[:,1]) 
    points = points[phi_index]
    xy_center = np.mean(points, axis=0)
    starts = []
    stops = []
    for pt in points.tolist(): # find the star and stop point for each edge point
        assert len(pt) == 3
        start = xy_center*1.0
        start[-1] = pt[-1]
        stop = start + l * (pt - start)
        starts.append(pt)
        stops.append(stop)
    faces = []
    for i in range(len(starts)):
        j = i+1 if i < len(starts)-1 else 0
        face1 = [3] + [i, i + len(starts)] + [j]
        face2 = [3] + [j, j + len(starts)] + [i + len(starts)]
        faces +=  face1
        faces += face2
    
    starts,stops = np.asarray(starts), np.asarray(stops)
    points = np.concatenate([starts,stops], axis=0)
    mesh = pyvista.PolyData(points, faces)
    return mesh

def get_points_id(lines):
    i = 0
    line_seq = []
    while i < len(lines):
        num = lines[i]
        point_seq = []
        for j in range(int(num)):
            point_seq.append(lines[i+1+j])
        i += num+1
        line_seq.append(point_seq)
    new_line_seq = [line_seq[0][0], line_seq[0][1]]
    for i in range(len(line_seq)):
        if i < len(line_seq) -1 :
            connect_point_mask = [(pt in line_seq[i]) for pt in line_seq[i+1] ]
            if True not in connect_point_mask:
               print('new pot:  ', line_seq[i], line_seq[i+1])
               final_pt = line_seq[i][1] if line_seq[i][0] in new_line_seq else line_seq[i][0]
               new_line_seq.append(final_pt)
               line_seq1 = new_line_seq.copy()
               new_line_seq = [line_seq[i+1][0],line_seq[i+1][1]]
            # new_line_seq.append(line_seq[0])
            # new_line_seq.append(line_seq[1])
            elif connect_point_mask[0] == connect_point_mask[1]:
                assert False
            else:
                connect_pt_id = np.where(~np.asarray(connect_point_mask))[0].item() # find the unseen point
                new_line_seq.append(line_seq[i+1][connect_pt_id]) 
    # temp = [p_seq for p_seq in line_seq[:i+1] if len(p_seq) > 2]
    # assert len(temp) == 0
    return line_seq1, new_line_seq

def find_edge_points(mesh, edge, epsilon = -1e-10):
    point_normals = mesh.point_normals
    z_directions = np.asarray([n[-1] >0 for n in point_normals])
    point_id  = np.where(z_directions == True)
    edge1_pointId, edge2_pointId= get_points_id(edge.lines)
    # edge_pointId = [Id for Id in edge_pointId if Id in point_id[0].tolist()]
    points1 = edge.points[edge1_pointId]
    points2 = edge.points[edge2_pointId]
    return points1, points2


def generate_cylinder(pv_implant):
    points = np.asarray(pv_implant.points)
    index = np.argsort(points[:,0])
    max_pt = points[index[-1]]
    max_x = max_pt[0]
    center_candidates = [pt for pt in points if np.math.sqrt((pt[0]-max_pt[0])**2 + (pt[1]-max_pt[1])**2)<1e-1]
    center_candidates = np.asarray(center_candidates)
    z_coord = np.mean(center_candidates[:,2])

    idx = pv_implant.find_closest_point((max_pt[0],max_pt[1],z_coord))
    return pv_implant.points[idx]

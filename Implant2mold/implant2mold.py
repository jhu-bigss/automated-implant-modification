import pyvista
import rhinoinside
rhinoinside.load()
import System
import Rhino
import numpy as np
from scipy import linalg as LA
from utils import *

class Implant2mold(object):
    def __init__(self, *params):
        self.params = params
    
    def PCA(self, points):
        #centering the data
        shift = np.mean(points, axis = 0)
        points = points -  shift
        cov = np.cov(points, rowvar = False)
        evals , evecs = LA.eigh(cov)
        idx = np.argsort(evals)[::-1]
        evecs = evecs[:,idx]
        evals = evals[idx]
        return evecs

    def compute_rotMat(self, points):
        evecs = self.PCA(points)
        x,y,z = evecs[:,:3]
        self.R = np.asarray([x,y,z], np.double)

    def get_min_box(self, points, faces):
        points_b = np.matmul(np.transpose(self.R), np.transpose(points))
        points_cloud = pyvista.PolyData(np.transpose(points_b), faces=faces)
        box = pyvista.Cube(bounds=points_cloud.bounds)
        rotated_box = np.matmul(self.R, np.transpose(np.asarray(box.points)))
        min_box = np.transpose(rotated_box) 
        min_box = pyvista.PolyData(min_box, faces = box.faces)
        return min_box, box, points_cloud
    

    def __call__(self,implant, s = 1.05):
        implant.save('orig_implant.ply')
        points = np.asarray(implant.points.copy())
        faces = implant.faces
        self.compute_rotMat(points)
        min_box, box, move_implant = self.get_min_box(points, faces)
        p = np.asarray(move_implant.points)
        p -= np.mean(p, axis = 0, keepdims=1)
        implant = pyvista.PolyData(p, faces=implant.faces)
        implant.flip_normals()
        bounds = np.asarray(implant.bounds).reshape(3,2)
        bounds[:,0] = (np.asarray(implant.center) - (bounds[:,1] - bounds[:,0])/2.0 * s)
        bounds[:,1] = (np.asarray(implant.center) + (bounds[:,1] - bounds[:,0])/2.0 * s)
        orig_box = pyvista.Cube(bounds=bounds.reshape(1,6)[0].tolist())
        z_max = bounds.reshape(1,6)[0][-1]
        bounds = np.asarray(implant.bounds).reshape(3,2)
        bounds[:,0] = (np.asarray(implant.center) - (bounds[:,1] - bounds[:,0])/2.0 * 1.4)
        bounds[:,1] = (np.asarray(implant.center) + (bounds[:,1] - bounds[:,0])/2.0 * 1.4)
        big_box = pyvista.Cube(bounds=bounds.reshape(1,6)[0].tolist())

        return mold, implant, orig_box, big_box

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
    
def ray_tracing(points, l = 5):
    points = np.asarray(points)
    polar_coords = Cartesian2Polar(points)
    phi_index= np.argsort(polar_coords[:,1])
    points = points[phi_index]
    xy_center = np.mean(points, axis=0)
    starts = []
    stops = []
    for pt in points.tolist():
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
    # mesh = pyvista.PolyData(points).delaunay_2d()
    return mesh

        
def pyvistaToTrimeshFaces(cells):
        faces = []
        idx = 0
        while idx < len(cells):
            curr_cell_count = cells[idx]
            curr_faces = cells[idx+1:idx+curr_cell_count+1]
            faces.append(curr_faces)
            idx += curr_cell_count+1
        return np.array(faces)



def remove_bounding_points(mold, mode = 'upper'):
    max_z = mold.bounds[-1]
    points = mold.points
    idx = np.argsort(np.asarray(points), axis = 0)[:,-1]
    idx = idx[::-1] if mode == 'upper' else idx
    mold, idxes = mold.remove_points(idx[:4])
    return mold



  
if __name__ == '__main__':
    pl = pyvista.Plotter()
    # implant = pyvista.read('implant-21-1932f-1.stl')
    # implant = pyvista.read('./data/implant-21-1932f-1_extrusion_5mm.ply')
    implant = pyvista.read('./data/test.stl')
    mold = Implant2mold()
    mold, implant,box, big_box = mold(implant, s=1.2)
    box.save('./data/box.stl')
    implant.flip_normals()
    mold = rhino_concatenate(box.triangulate(),implant)
    # mold = trimesh_cut(box.triangulate(), implant)
    mold =  pyvista.read('./data/mold.stl')
    
    edges = implant.extract_feature_edges(30)
    edge_points_bottom = edges.connectivity(largest=True)
    largest_points = edge_points_bottom.points[edge_points_bottom.active_scalars==0]
    # edge_points_bottom, edge_points_top = find_edge_points(implant, edges)
    cut_plane = ray_tracing(largest_points) # generate the cut_plane with hole
    bottom_surf = split_surf(implant, mode='bottom')  # find the top surface of implant
    bottom_mold_surf = rhino_concatenate(cut_plane, bottom_surf)
    Bottom_mold = rhino_concatenate(rhino_splitv2(box.triangulate(), bottom_mold_surf),\
        rhino_splitv2(bottom_mold_surf, box.triangulate()))
    pl.add_mesh(Bottom_mold, color='blue')
    skirt_top_surf = rhino_splitv2(implant, cut_plane)
    top_mold_surf = rhino_concatenate(cut_plane, skirt_top_surf)
    # pl.add_mesh(implant)
    Top_mold = rhino_concatenate(rhino_splitv2(box.triangulate(), top_mold_surf,1),\
        rhino_splitv2(top_mold_surf, box.triangulate()))
    cut_plane.save('./data/cut_plane.stl')
    implant.save('./data/reoriented_implant.stl')
    pl.add_mesh(Top_mold)
    pl.show()
    

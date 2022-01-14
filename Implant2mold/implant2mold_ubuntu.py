import pyvista
import numpy as np
from scipy import linalg as LA
from utils_trimesh import split_surf, generate_cut_surf,pyvistaToTrimeshFaces,\
    remove_bounding_points, Cartesian2Polar
import trimesh

class Reorient(object):
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
    
    def reorient(self, mesh):
        min_box, box, points_cloud = self.get_min_box(np.asarray(mesh.points),mesh.faces)
        p = np.asarray(points_cloud.points)
        p -= np.mean(p, axis = 0, keepdims=1)
        mesh = pyvista.PolyData(p, faces=points_cloud.faces)
        mesh.flip_normals()
        return mesh

    def __call__(self,implant, s = 1.05):
        # implant.save('orig_implant.ply')
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

        return implant, orig_box

    



  
if __name__ == '__main__':
    top_surf = pyvista.read('./data/reoriented_implant_top.stl')
    bottom_surf = pyvista.read('./data/reoriented_implant_bottom.stl')
    # NotImplemented reorient the surface
    # reorient = Reorient()
    # top_surf, box = reorient(top_surf,s=1.2)
    edges = bottom_surf.extract_feature_edges(30)

    #find the largest edge
    edge_points_bottom = edges.connectivity(largest=True)
    largest_points = edge_points_bottom.points[edge_points_bottom.active_scalars==0]

    cut_plane = generate_cut_surf(largest_points) # geberate cut surface
    bottom_mold = cut_plane.merge(bottom_surf)
    bottom_mold = bottom_mold.extrude([0,0,70],capping=True)
    top_mold = cut_plane.merge(top_surf)
    top_mold = top_mold.extrude([0,0,-70],capping=True)
    box1 =  pyvista.read('./data/box.stl') # read the reoriented bounding box

    # boolean cut
    box = trimesh.Trimesh(np.asarray(box1.triangulate().points), faces = pyvistaToTrimeshFaces(box1.faces))
    bottom_mold = trimesh.Trimesh(np.asarray(bottom_mold.triangulate().points), faces = pyvistaToTrimeshFaces(bottom_mold.faces))
    bottom_mold = trimesh.boolean.difference([box, bottom_mold])
    box = trimesh.Trimesh(np.asarray(box1.triangulate().points), faces = pyvistaToTrimeshFaces(box1.faces))
    top_mold = trimesh.Trimesh(np.asarray(top_mold.triangulate().points), faces = pyvistaToTrimeshFaces(top_mold.faces))
    top_mold = trimesh.boolean.difference([box, top_mold])

    # remove the end points and visualize
    pl = pyvista.Plotter()
    pl.add_mesh(remove_bounding_points(pyvista.wrap(bottom_mold)))
    pl.add_mesh(remove_bounding_points(pyvista.wrap(top_mold), mode = 'bottom'),color='blue')
    pl.show()

import numpy as np
import cmath
import rhinoinside
rhinoinside.load()
import pyvista as pv
from System.Collections import Generic as grc
import Rhino

"""
utility functions including transformation from pyvista mesh to rhino mesh or inverse;
mesh_cut based on rhino
BooleanSplit
"""
def pv2mesh(pv_mesh):
    """
    convert pyvista mesh to Rhino mesh
    """
    mesh = Rhino.Geometry.Mesh()
    cells = pv_mesh.faces
    faces = []
    idx = 0
    while idx < len(cells):
        curr_cell_count = cells[idx]
        curr_faces = cells[idx+1:idx+curr_cell_count+1]
        faces.append(curr_faces)
        idx += curr_cell_count+1
    faces = np.asarray(faces)
    vertices = np.asarray(pv_mesh.points)
    for vertice in vertices:
        mesh.Vertices.Add(vertice[0],vertice[1],vertice[2])
    for face in faces:
        if len(face) == 4:
            mesh.Faces.AddFace(face[0],face[1],face[2],face[3])
        elif len(face) == 3:
            mesh.Faces.AddFace(face[0],face[1],face[2])
    return mesh


def wrap2pv(rhino_mesh):
    """
    warp rhino mesh to pyvista
    """
    faces = []
    for face in rhino_mesh.Faces:
        faces += [3]+[face[i] for i in range(3)]
    faces = np.asarray(faces)
    points = []
    for vertice in rhino_mesh.Vertices:
        points += [[vertice.X, vertice.Y, vertice.Z]]  
    return pv.PolyData(points, faces = faces)
    
def rhino_cut(mesh1, mesh2):
    """
    Rhinot CreateBooleanDifference
    """
    rhino_mesh1, rhino_mesh2 = pv2mesh(mesh1), pv2mesh(mesh2)
    mesh1 =grc.List[Rhino.Geometry.Mesh]()
    mesh1.Add(rhino_mesh1)
    mesh2 = grc.List[Rhino.Geometry.Mesh]()
    mesh2.Add(rhino_mesh2)
    cut = Rhino.Geometry.Mesh.CreateBooleanDifference(mesh1, mesh2)
    return warp2pv(cut[0])

def rhino_concatenate(mesh1,mesh2):
    """
    Rhinot AppendMesh
    """
    rhino_mesh1, rhino_mesh2 = pv2mesh(mesh1), pv2mesh(mesh2)
    rhino_mesh1.Append(rhino_mesh2)
    return warp2pv(rhino_mesh1)

def rhino_split(mesh, surface):
    """
    Rhinot CreateBooleanSplit
    """
    rhino_mesh, rhino_surf = pv2mesh(mesh), pv2mesh(surface)
    mesh =grc.List[Rhino.Geometry.Mesh]()
    mesh.Add(rhino_mesh)
    surf = grc.List[Rhino.Geometry.Mesh]()
    surf.Add(rhino_surf)
    cut = Rhino.Geometry.Mesh.CreateBooleanSplit(mesh, surf)
    return warp2pv(cut[0])

def rhino_splitv2(mesh, surface,n=0):
    """
    Rhino Split

    Prameters
    ----------
    mesh: pyvista Datafilter mesh, mesh to be splited
    surface:  pyvista Datafilter, surface to split the mesh
    n      : the index of the splitted submesh to return 
    """
    rhino_mesh, rhino_surf = pv2mesh(mesh), pv2mesh(surface)

    cut = Rhino.Geometry.Mesh.Split(rhino_mesh, rhino_surf)
    return warp2pv(cut[n])

def split_surf(implant, mode):
    """
    find the bottom or top surface of the implant
    """
    normals = np.asarray(implant.point_normals)
    mask = normals[:,2] > 0.1 if mode == 'bottom' else normals[:,2] < -0.1
    idxes = np.arange(len(normals))[~mask]
    splited_surf,_ = implant.remove_points(idxes)
    
    return splited_surf

def Cartesian2Polar(coords):
    polar_coords = []
    for coord in coords:
        coord_xy = coord[:-1]
        cn = complex(coord_xy[0], coord_xy[1])
        r,phi = cmath.polar(cn) 
        polar_coords.append([r, phi])
    return np.asarray(polar_coords)


if __name__=='__main__':
    plot = pv.Plotter()
    pv_mesh = pv.Cube()
    pv_mesh2 = pv.Sphere(0.6)
    meshes = grc.List[Rhino.Geometry.Mesh]()
    rhino_mesh = pv2mesh(pv_mesh)
    meshes.Add(rhino_mesh)
    # plot.add_mesh(warp2pv(meshes[0]))
    # plot.show()
    rhino_cut(pv_mesh,pv_mesh2).plot()
    coords = [[1,1,1],[2,2,1],[1,-1,1]]
    print(Cartesian2Polar(coords))

# import rhinoinside
# rhinoinside.load()
# import System
# import Rhino
import pyvista as pv
from utils_trimesh import split_surf, generate_cut_surf,pyvistaToTrimeshFaces,\
    remove_bounding_points
import trimesh
import numpy as np

# for now, you need to explicitly use floating point
# numbers in Point3d constructor
# pts = System.Collections.Generic.List[Rhino.Geometry.Point3d]()
# pts.Add(Rhino.Geometry.Point3d(0.0,0.0,0.0))
# pts.Add(Rhino.Geometry.Point3d(1.0,0.0,0.0))
# pts.Add(Rhino.Geometry.Point3d(1.5,2.0,0.0))

# crv = Rhino.Geometry.Curve.CreateInterpolatedCurve(pts,3)
# print (crv.GetLength())

bottom_surf = pv.read('./data/reoriented_implant_bottom.stl')
edges = bottom_surf.extract_feature_edges(30)
edge_points_bottom = edges.connectivity(largest=True)
largest_points = edge_points_bottom.points[edge_points_bottom.active_scalars==0]
cut_plane = generate_cut_surf(largest_points)
bottom = cut_plane.merge(bottom_surf)
bottom = bottom.extrude([0,0,70],capping=True)
top = cut_plane.merge(pv.read('./data/reoriented_implant_top.stl'))
top = top.extrude([0,0,-70],capping=True)
box =  pv.read('./data/box.stl')
box = trimesh.Trimesh(np.asarray(box.triangulate().points), faces = pyvistaToTrimeshFaces(box.faces))
bottom = trimesh.Trimesh(np.asarray(bottom.triangulate().points), faces = pyvistaToTrimeshFaces(bottom.faces))
bottom = trimesh.boolean.difference([box, bottom])
pl = pv.Plotter()
# pl.add_mesh(pv.wrap(bottom))
pl.add_mesh(remove_bounding_points(pv.wrap(bottom),'bottom'))
pl.show()

import pyvista as pv
import pymeshlab as pm

import utils

ms = pm.MeshSet()
ms.load_project('data/test.mlp')

print(ms.print_status())

meshes = []
plotter = pv.Plotter()

for i in range(ms.number_meshes()):
    mesh_pv = utils.meshlab2pv(ms.mesh(i))
    meshes.append(mesh_pv)
    plotter.add_mesh(mesh_pv)


plotter.add_mesh_slice_orthogonal(meshes[0])

plotter.show()
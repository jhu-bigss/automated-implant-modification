import numpy as np
import pyvista as pv


def meshlab2pv(mmesh):
    '''
    Convert meshlab mesh to PyVista polydata
    '''
    mpoints, mcells = mmesh.vertex_matrix(), mmesh.face_matrix()

    if len(mcells):
        # convert the faces into PolyData format
        mfaces = []
        for cell in mcells:
            face = np.hstack((len(cell), cell))
            mfaces.extend(face.tolist())
        mfaces = np.array(mfaces)
        polydata = pv.PolyData(mpoints, np.hstack(mfaces))
    else:
        polydata = pv.PolyData(mpoints, None)

    return polydata

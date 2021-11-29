import sys

from PyQt5 import Qt, QtCore
import numpy as np

import pyvista as pv
from pyvistaqt import QtInteractor

# picking 3 points to define a coordinate system
P0_ORIGIN = 0
P1_X_POSITIVE = 1
P2_Y_POSITIVE = 2

axes_length = 15 # mm

class MainWidget(Qt.QWidget):

    def __init__(self, parent=None, show=True):
        super(MainWidget, self).__init__()

        self.load_button = Qt.QPushButton("Load")
        self.transform_button = Qt.QPushButton("Transform")
        self.save_button = Qt.QPushButton("Save")

        self.load_button.clicked.connect(self.load_mesh_event)
        self.transform_button.clicked.connect(self.transform_coord_sys)
        self.save_button.clicked.connect(self.save_mesh)

        self.frame = Qt.QFrame()
        self.plotter = QtInteractor(self.frame)
        vlayout = Qt.QVBoxLayout()
        vlayout.addWidget(self.plotter.interactor)
        hlayout_1 = Qt.QHBoxLayout()
        hlayout_1.addWidget(self.load_button)
        hlayout_1.addWidget(self.transform_button)
        hlayout_1.addWidget(self.save_button)
        vlayout.addLayout(hlayout_1)
        self.setLayout(vlayout)

        self.setWindowTitle("Define New Coordinate System")
        self.setGeometry(550, 200, 800, 600)

        # Enable dragging and dropping onto the GUI
        self.setAcceptDrops(True)

        self.plotter.show_axes()
        self.plotter.enable_point_picking(self.point_picking_event, show_message=False, show_point=False)
        self.mesh = None

        # start picking the 1st point which is ORIGIN
        self.point_picking = P0_ORIGIN
        self.axes_plotted = False
        self.transformation_matrix = None

        if show:
            self.show()

    # The following three methods set up dragging and dropping for the app
    def dragEnterEvent(self, e):
        if e.mimeData().hasUrls:
            e.accept()
        else:
            e.ignore()

    def dragMoveEvent(self, e):
        if e.mimeData().hasUrls:
            e.accept()
        else:
            e.ignore()

    def dropEvent(self, e):
        """
        Drop files directly onto the widget
        File locations are stored in fname
        :param e:
        :return:
        """
        if e.mimeData().hasUrls:
            e.setDropAction(QtCore.Qt.CopyAction)
            e.accept()
            # Workaround for OSx dragging and dropping
            for url in e.mimeData().urls():
                fname = str(url.toLocalFile())
            self.fname = fname
            self.load_mesh()
        else:
            e.ignore()

    def load_mesh_event(self):
        """
        Open a mesh file
        """
        self.fname, _ = Qt.QFileDialog.getOpenFileName(self, 'Open file','',"(*.ply) ;; (*.stl)")
        self.load_mesh()

    def load_mesh(self):
        self.mesh = pv.read(self.fname)
        self.plotter.clear()
        self.plotter.add_mesh(self.mesh, rgb=True, show_scalar_bar=False)
        self.plotter.reset_camera()

    def save_mesh(self):
        """
        Save mesh
        """
        self.fname, f_filter = Qt.QFileDialog.getSaveFileName(self, 'Save file', self.fname, "(*.ply) ;; (*.stl)")
        pv.save_meshio(self.fname, self.mesh, file_format=f_filter.strip('(*.)'))
        # close the window
        self.close()

    def point_picking_event(self, point):
        # check which one is picking then move onto next one
        if self.point_picking is P0_ORIGIN:
            self.point_picking = P1_X_POSITIVE
            return self.plotter.add_sphere_widget(callback=self.picked_p0, center=point, radius=2, pass_widget=True)
        if self.point_picking is P1_X_POSITIVE:
            self.point_picking = P2_Y_POSITIVE
            return self.plotter.add_sphere_widget(callback=self.picked_p1, center=point, radius=1, pass_widget=True)
        if self.point_picking is P2_Y_POSITIVE:
            self.point_picking = None
            return self.plotter.add_sphere_widget(callback=self.picked_p2, center=point, radius=1, pass_widget=True)
        print("already picked all 3 points.")

    def picked_p0(self, point, widget):
        # find the closest point on mesh to the sphere widget; update the widget center
        point = self.mesh.points[self.mesh.find_closest_point(point)]
        print("defines O: ", point)
        self.p0 = widget
        self.p0.SetCenter(point)
        # plot the axes
        self.update_axes()
        
    def picked_p1(self, point, widget):
        point = self.mesh.points[self.mesh.find_closest_point(point)]
        print("defines X: ", point)
        self.p1 = widget
        self.p1.SetCenter(point)
        self.update_axes()
        
    def picked_p2(self, point, widget):
        point = self.mesh.points[self.mesh.find_closest_point(point)]
        print("defines Y: ", point)
        self.p2 = widget
        self.p2.SetCenter(point)
        self.update_axes()

    def update_axes(self):
        # draw XYZ axes once finsh points picking
        if not self.point_picking:
            # clear the existing axes actors
            if self.axes_plotted:
                self.clear_axes()
            # construte new coord sys from picked points sphere widget
            origin, x, y, z = self.construct_coord_sys(np.asarray(self.p0.GetCenter()),
                                                       np.asarray(self.p1.GetCenter()),
                                                       np.asarray(self.p2.GetCenter())
                                                       )
            self.plotter.add_arrows(origin, x, mag=axes_length, color='Red', name='x', reset_camera=False)
            self.plotter.add_arrows(origin, y, mag=axes_length, color='Green', name='y', reset_camera=False)
            self.plotter.add_arrows(origin, z, mag=axes_length, color='Blue', name='z', reset_camera=False)
            self.axes_plotted = True
            # update transformation
            self.transformation_matrix = np.array([x, y, z, origin]).T
            self.transformation_matrix = np.append(self.transformation_matrix, [[0,0,0,1]], axis=0)
    
    def clear_axes(self):
        self.plotter.remove_actor('x')
        self.plotter.remove_actor('y')
        self.plotter.remove_actor('z')
        self.axes_plotted = False
    
    @staticmethod
    def construct_coord_sys(p0, p1, p2):
        origin = p0
        x_axis = p1 - origin
        x_axis = x_axis/np.linalg.norm(x_axis)
        p2_vec = p2 - origin
        p2_projected_on_x_axis = np.dot(p2_vec, x_axis) * x_axis + origin
        y_axis = p2 - p2_projected_on_x_axis
        y_axis = y_axis/np.linalg.norm(y_axis)
        z_axis = np.cross(x_axis, y_axis)
        
        return origin, x_axis, y_axis, z_axis
        
    def transform_coord_sys(self):
        if self.transformation_matrix is not None:
            self.mesh.transform(np.linalg.inv(self.transformation_matrix))
            self.clear_axes()
            self.plotter.clear_sphere_widgets()
            self.plotter.add_axes_at_origin(labels_off=True)
            self.plotter.reset_camera()

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window = MainWidget()
    sys.exit(app.exec_())
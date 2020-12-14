import os, sys

from PyQt5 import Qt, QtCore
import numpy as np

import pyvista as pv
from pyvistaqt import QtInteractor

# mode
ADJUST_ANGLE_THRESHOLD = 0
SELECT_EDGE = 1

class MainWidget(Qt.QWidget):

    def __init__(self, parent=None, show=True):
        super(MainWidget, self).__init__()

        # ui design
        self.load_button = Qt.QPushButton("Load")
        self.extract_edge_button = Qt.QPushButton("Extract feature edges")

        self.load_button.clicked.connect(self.load_mesh_event)
        self.extract_edge_button.clicked.connect(self.extract_edge_event)

        self.frame = Qt.QFrame()
        self.plotter = QtInteractor(self.frame)
        vlayout_main = Qt.QVBoxLayout()
        vlayout_main.addWidget(self.plotter.interactor)
        hlayout_1 = Qt.QHBoxLayout()
        hlayout_1.addWidget(self.load_button)
        hlayout_1.addWidget(self.extract_edge_button)
        vlayout_main.addLayout(hlayout_1)
        self.setLayout(vlayout_main)

        self.setWindowTitle("Extract defect contour ground truth")
        self.setGeometry(500, 150, 1000, 800)

        # Enable dragging and dropping onto the GUI
        self.setAcceptDrops(True)

        # initialize
        self.plotter.show_axes()
        self.mesh = None
        self.mode = ADJUST_ANGLE_THRESHOLD
        self.edge = None

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
        if self.mesh is not None:
            self.plotter.remove_actor('mesh')
        self.mesh = pv.read(self.fname)
        self.plotter.add_mesh(self.mesh, rgb=True, show_scalar_bar=False, name='mesh')
        self.plotter.reset_camera()

    def save_edge(self, edge):
        """
        Save the selected edge
        """
        f_name, f_extension = Qt.QFileDialog.getSaveFileName(self, 'Save selected edge as', self.fname.strip(self.fname.split('/')[-1]), "(*.vtk) ;; (*.csv)")
        f_extension = f_extension.strip('(*.)')
        if f_extension == 'vtk':
            edge.save(f_name)
        else:
            np.savetxt(f_name, edge.points)

    def extract_edge_event(self):
        if self.mode is ADJUST_ANGLE_THRESHOLD:
            # 1. adjust angle threshhold to filter out certain edges
            self.plotter.remove_actor('mesh')
            self.plotter.add_slider_widget(self.angle_threshold_slider_widget_callback, rng=(0,90), value=30, title='Threshold Angle')
            self.extract_edge_button.setText("Confirm")
            self.mode += 1
        elif self.mode is SELECT_EDGE:
            # 2. split the bodies into multiBlock, then select the desired edge
            self.edges = self.edges.split_bodies()
            self.plotter.clear_slider_widgets()
            self.plotter.enable_point_picking(self.point_picking_callback, use_mesh=True, show_message=False, show_point=False)
            self.extract_edge_button.setText("Save the selected edge")
            self.mode += 1
        else:
            # 3. save the selected edge
            if self.edge is not None:
                self.extract_edge_button.setText("Saved")
                self.save_edge(self.edge)
            else:
                print("Please select an edge in the scene using keyboard \"P\"")
        
    def angle_threshold_slider_widget_callback(self, value):
        self.edges = self.mesh.extract_feature_edges(feature_angle=value, boundary_edges=False)
        self.plotter.add_mesh(self.edges, show_scalar_bar=False, name='edges')
        
    def point_picking_callback(self, selected_mesh, point_id):
        # loop through to find which block of the splited body that the user has picked
        if selected_mesh is not None:
            for i in range(self.edges.n_blocks):
                if selected_mesh.points[point_id] in self.edges[i].points:
                    self.edge = self.edges[i]
                    self.plotter.add_mesh(self.edge, color='Red', show_scalar_bar=False, name='edge')

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window = MainWidget()
    sys.exit(app.exec_())
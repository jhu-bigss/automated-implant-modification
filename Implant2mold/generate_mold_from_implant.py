import sys

from PyQt5 import Qt, QtCore

import pyvista as pv
from pyvistaqt import QtInteractor
from implant2mold import Implant2mold
# from pyvista import themes
# pv.set_plot_theme(themes.DarkTheme())


class MainWidget(Qt.QWidget):

    def __init__(self, parent=None, show=True):
        super(MainWidget, self).__init__()

        self.test_button = Qt.QPushButton("Open")
        self.test_button.clicked.connect(self.test_button_event)

        self.frame = Qt.QFrame()
        self.plotter = QtInteractor(self.frame)
        vlayout = Qt.QVBoxLayout()
        vlayout.addWidget(self.plotter.interactor)
        hlayout = Qt.QHBoxLayout()
        hlayout.addWidget(self.test_button)
        vlayout.addLayout(hlayout)
        self.setLayout(vlayout)

        self.setWindowTitle("Test Qt Window")
        self.setGeometry(550, 200, 800, 600)
        self.mold_generator = Implant2mold()
        # Enable dragging and dropping onto the GUI
        self.setAcceptDrops(True)

        self.mesh = None
        self.plotter.show_axes()

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

    def test_button_event(self):
        """
        Open a mesh file
        """
        self.fname, _ = Qt.QFileDialog.getOpenFileName(self, 'Open file','',"(*.ply) ;; (*.stl)")
        self.load_mesh()

    def load_mesh(self):
        self.mesh = pv.read(self.fname)
        self.mesh = self.mesh.smooth(120)
        self.mesh = self.mold_generator(self.mesh)
        self.plotter.clear()
        
        self.plotter.add_mesh(self.mesh)
     
        self.plotter.add_slider_widget(callback=self.adjust_feature_edge_extraction, rng=(0, 180))
        self.plotter.enable_point_picking(callback=self.select_feature_edges, show_message=True, use_mesh=True)

    def save_mesh(self):
        """
        Save mesh
        """
        self.fname, f_filter = Qt.QFileDialog.getSaveFileName(self, 'Save file', self.fname, "(*.ply) ;; (*.stl)")
        pv.save_meshio(self.fname, self.mesh, file_format=f_filter.strip('(*.)'))
        # close the window
        self.close()

    def adjust_feature_edge_extraction(self, value):
        self.edges = self.mesh.extract_feature_edges(feature_angle=value, boundary_edges=False, non_manifold_edges=False)
        self.plotter.add_mesh(self.edges, color="red", line_width=5, name="edges")

    def select_feature_edges(self, picked_mesh, picked_point_id):
        point = picked_mesh.points[picked_point_id]
        print(point)

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window = MainWidget()
    sys.exit(app.exec_())
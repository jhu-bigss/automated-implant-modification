import sys

from PyQt5 import Qt, QtCore

import pyvista as pv
from pyvistaqt import QtInteractor

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

        # Enable dragging and dropping onto the GUI
        self.setAcceptDrops(True)

        self.plotter.show_axes()
        self.mesh = None

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
        self.plotter.clear()
        self.plotter.add_mesh(self.mesh, show_edges=True)
        # self.plotter.reset_camera()
        self.plotter.add_slider_widget(self.adjust_feature_edge_extraction, rng=(0, 180))

    def save_mesh(self):
        """
        Save mesh
        """
        self.fname, f_filter = Qt.QFileDialog.getSaveFileName(self, 'Save file', self.fname, "(*.ply) ;; (*.stl)")
        pv.save_meshio(self.fname, self.mesh, file_format=f_filter.strip('(*.)'))
        # close the window
        self.close()

    def adjust_feature_edge_extraction(self, value):
        self.edges = self.mesh.extract_feature_edges(value, non_manifold_edges=False)
        self.plotter.add_mesh(self.edges, color="red", line_width=5, name="edges")

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window = MainWidget()
    sys.exit(app.exec_())

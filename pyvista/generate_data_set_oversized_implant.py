import os, sys

from PyQt5 import Qt, QtCore
import numpy as np

import pyvista as pv
from pyvistaqt import QtInteractor
import pymeshfix

class MainWidget(Qt.QWidget):

    def __init__(self, parent=None, show=True):
        super(MainWidget, self).__init__()

        # ui design
        self.load_button = Qt.QPushButton("Load")
        self.spline_resolution_label = Qt.QLabel("Resolution")
        self.spline_resolution_spinbox = Qt.QSpinBox()
        self.spline_resolution_spinbox.setMaximum(1000)
        self.spline_resolution_spinbox.setValue(100)
        self.spline_clip_button = Qt.QPushButton("Create Spline")
        self.save_button = Qt.QPushButton("Save")

        self.load_button.clicked.connect(self.load_mesh_event)
        self.spline_clip_button.clicked.connect(self.spline_skirt_clip_event)
        self.save_button.clicked.connect(self.save_mesh)

        self.frame = Qt.QFrame()
        self.plotter = QtInteractor(self.frame)
        vlayout_main = Qt.QVBoxLayout()
        vlayout_main.addWidget(self.plotter.interactor)
        hlayout_1 = Qt.QHBoxLayout()
        hlayout_1.addWidget(self.load_button)
        hlayout_2 = Qt.QHBoxLayout()
        hlayout_2.addWidget(self.spline_resolution_label)
        hlayout_2.addWidget(self.spline_resolution_spinbox)
        hlayout_1.addLayout(hlayout_2)
        hlayout_1.addWidget(self.spline_clip_button)
        hlayout_1.addWidget(self.save_button)
        vlayout_main.addLayout(hlayout_1)
        self.setLayout(vlayout_main)

        self.setWindowTitle("Generate experiment data set - implant only")
        self.setGeometry(500, 150, 1000, 800)

        # Enable dragging and dropping onto the GUI
        self.setAcceptDrops(True)

        # initialize
        self.plotter.show_axes()
        self.plotter.enable_point_picking(self.point_picking_event, show_message=False, show_point=False)
        self.plotter.enable_depth_peeling()
        self.mesh = None
        self.implant_sphere_widgets = []
        self.implant_spline_widget = None
        self.line_widget = None
        self.line_widget_point_2 = None
        self.extrude_shrink_ratio = 0
        self.implant_spline_skirt = None

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
            self.load_button.setText("Reset")
        else:
            e.ignore()

    def load_mesh_event(self):
        """
        Open a mesh file
        """
        if self.mesh is None:
            self.fname, _ = Qt.QFileDialog.getOpenFileName(self, 'Open file','',"(*.ply) ;; (*.stl)")
            self.load_button.setText("Reset")
        else:
            self.reset_spline_skirt_widget()
        self.load_mesh()

    def load_mesh(self):
        if self.mesh is not None:
            self.plotter.remove_actor('mesh')
        self.mesh = pv.read(self.fname)
        self.plotter.add_mesh(self.mesh, rgb=True, show_scalar_bar=False, name='mesh')
        self.plotter.reset_camera()

    def save_mesh(self):
        """
        Save generated implant mesh
        """
        f_name, f_filter = Qt.QFileDialog.getSaveFileName(self, 'Save implant', self.fname, "(*.stl) ;; (*.ply)")
        pv.save_meshio(f_name, self.implant_mesh, file_format=f_filter.strip('(*.)'))

    
    def point_picking_event(self, point):
        if self.implant_spline_widget is None:
            return self.plotter.add_sphere_widget(callback=self.implant_sphere_widget_callback, center=point, radius=2, pass_widget=True)
        else:
            print("The clip spline is already created, point picking is not allowed")

    def implant_sphere_widget_callback(self, point, widget):
        if widget in self.implant_sphere_widgets:
            widget.SetCenter(point)
        else:
            self.implant_sphere_widgets.append(widget)

    def spline_skirt_clip_event(self):
        if self.implant_spline_widget is None:
            # 1. create implant spline skirt
            self.plotter.add_spline_widget(self.implant_spline_callback, n_hanldes=len(self.implant_sphere_widgets), resolution=self.spline_resolution_spinbox.value(), pass_widget=True)
            if self.line_widget is None:
                self.plotter.add_slider_widget(self.slider_widget_callback, rng=(0,1), value=self.extrude_shrink_ratio)
                self.plotter.add_line_widget(self.line_widget_callback, pass_widget=True)
            self.spline_resolution_spinbox.setEnabled(False)
            self.spline_clip_button.setText("Cut")
        else:
            # 2. clip the mesh by the spline skirt to generate data set
            self.spline_clip_button.setEnabled(False)
            self.clip_mesh_by_spline_skirt()

    def implant_spline_callback(self, spline_polydata, widget):
        # 1. creant spline widget
        if self.implant_spline_widget is None:
            # create spline given handle positions and make it CLOSED
            for i, sphere in enumerate(self.implant_sphere_widgets):
                widget.SetHandlePosition(i, *sphere.GetCenter())
            if not widget.GetClosed():
                widget.ClosedOn()
            self.implant_spline_widget = widget
            # disable sphere widgets once spline being created
            self.plotter.clear_sphere_widgets()
        else:
            # 2. manipulate spline widget --> update line widget
            self.line_widget_point_1 = spline_polydata.center
            self.line_widget.SetPoint1(self.line_widget_point_1)
            self.update_spline_skirt()
        
    def slider_widget_callback(self, value):
        self.extrude_shrink_ratio = value
        self.update_spline_skirt()
        
    def line_widget_callback(self, line_polydata, widget):
        # 1. create line widget
        if self.line_widget is None:
            # grab the spline polydata
            spline_polydata = pv.PolyData()
            self.implant_spline_widget.GetPolyData(spline_polydata)
            widget.SetPoint1(spline_polydata.center)
            if self.line_widget_point_2 is not None:
                widget.SetPoint2(self.line_widget_point_2)
            else:
                widget.SetPoint2(self.mesh.center)
            self.line_widget = widget
            self.line_widget_point_1 = np.asarray(widget.GetPoint1())
            self.line_widget_point_2 = np.asarray(widget.GetPoint2())
        else:
            # 2. manipulate line widget --> update spline widget
            rotate_center = np.asarray(widget.GetPoint2())
            rotate_sphere_handle = np.asarray(widget.GetPoint1())
            if not np.array_equal(rotate_sphere_handle, self.line_widget_point_1):
                # update spline handles only when point_1 is modified
                vec_old = self.line_widget_point_1 - rotate_center
                vec_new = rotate_sphere_handle - rotate_center
                scalar_translate = np.linalg.norm(vec_new) - np.linalg.norm(vec_old)
                vec_new_norm = vec_new / np.linalg.norm(vec_new)
                vec_translate = scalar_translate * vec_new_norm
                R = self.rotation_matrix_from_vectors(vec_old, vec_new)
                # update spline widget handles
                self.transform_spline_widget(self.implant_spline_widget, rotate_center, vec_translate, R)
            # update line widget point 1
            self.line_widget_point_1 = rotate_sphere_handle
        self.update_spline_skirt()
        
    def update_spline_skirt(self):
        # grab the spline widget polydata and the line widget polydata
        vec_extrude = np.asarray(self.line_widget.GetPoint2()) - np.asarray(self.line_widget.GetPoint1())
        
        # implant spline skirt
        if self.implant_spline_widget is not None:
            implant_spline_polydata = pv.PolyData()
            self.implant_spline_widget.GetPolyData(implant_spline_polydata)
            self.implant_spline_skirt = implant_spline_polydata.extrude(vec_extrude)
            
            lower_polydata = self.implant_spline_skirt.points[len(implant_spline_polydata.points):]
            vectors = np.asarray(self.line_widget.GetPoint2()) - lower_polydata
            offset = self.extrude_shrink_ratio * vectors
            updated_lower_polydata = lower_polydata + offset
            self.implant_spline_skirt.points[len(implant_spline_polydata.points):] = updated_lower_polydata
        
            self.plotter.add_mesh(self.implant_spline_skirt, color='Blue', name='implant_spline_skirt')

    def clip_mesh_by_spline_skirt(self):
        # Note: need to save then to read it again to make the surface manifold with all face normals consistent facing outward
        self.implant_spline_skirt.fill_holes(100, inplace=True)
        self.implant_spline_skirt.flip_normals()
        self.implant_spline_skirt.save("temp_implant_spline_skirt.stl")
        implant_spline_skirt = pv.read("temp_implant_spline_skirt.stl")
        implant_spline_skirt.flip_normals()
        
        # note: boolean operation is very computational expensive
        self.implant_mesh = self.mesh.boolean_cut(implant_spline_skirt)
        
        # fix mesh manifold using pymeshfix
        self.implant_mesh.save("temp_implant.stl")
        implant_mesh = pv.read("temp_implant.stl")
        meshfix = pymeshfix.MeshFix(implant_mesh)
        meshfix.repair(verbose= True, joincomp=True, remove_smallest_components=False)
        self.implant_mesh = meshfix.mesh
        
        # update plot and remove generated temp files
        self.clear_all_widgets()
        os.remove("temp_implant_spline_skirt.stl")
        os.remove("temp_implant.stl")
        self.plotter.add_mesh(self.implant_mesh, show_scalar_bar=False, name='mesh')
        
    def reset_spline_skirt_widget(self):
        self.clear_all_widgets()
        self.spline_clip_button.setText("Create Spline")
        self.spline_clip_button.setEnabled(True)
        self.spline_resolution_spinbox.setEnabled(True)
        
        self.implant_sphere_widgets = []
        self.implant_spline_widget = None
        self.implant_spline_skirt = None
        self.line_widget_point_2 = None
        self.line_widget = None
        
    def clear_all_widgets(self):
        self.plotter.clear()
        self.plotter.clear_spline_widgets()
        self.plotter.clear_line_widgets()
        self.plotter.clear_slider_widgets()
    
    def rotation_matrix_from_vectors(self, vec1, vec2):
        """ Find the rotation matrix that aligns vec1 to vec2
        :param vec1: A 3d "source" vector
        :param vec2: A 3d "destination" vector
        :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
        :ref: https://stackoverflow.com/questions/45142959/calculate-rotation-matrix-to-align-two-vectors-in-3d-space
        """
        a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return R
    
    def transform_spline_widget(self, widget, rotate_center, translation, R):
        """ transform the spline widget
        :param widget: the spline widget to get its handles
        :param rotate_center: the line widget point_2
        :param translation: along the new axis
        :param R: 3x3 rotation matrix
        """
        n_handles = widget.GetNumberOfHandles()
        for i in range(n_handles):
            vec = np.asarray(widget.GetHandlePosition(i)) - rotate_center
            vec = np.matmul(R, vec)
            point_new = vec + translation + rotate_center
            widget.SetHandlePosition(i, *point_new)


if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window = MainWidget()
    sys.exit(app.exec_())
import sys, os, vtk
from functools import reduce

import pyvista as pv
import numpy as np
import trimesh

from PyQt5 import Qt, QtCore, uic
from pyvistaqt import QtInteractor


class MainWindow(Qt.QMainWindow):

    def __init__(self, parent=None, show=True):
        Qt.QMainWindow.__init__(self, parent)
        ui = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ui/toolpath_generation.ui')
        self.ui = uic.loadUi(ui, self)

        # add the pyvista interactor object
        self.plotter = QtInteractor(self.vtk_frame)
        vtk_layout = Qt.QHBoxLayout()
        vtk_layout.addWidget(self.plotter.interactor)
        self.vtk_frame.setLayout(vtk_layout)

        # menuFile, menuMesh
        self.load_mesh_action.triggered.connect(self.load_mesh_event)
        meshMenu = self.menubar.addMenu('Mesh')
        self.reset_input_mesh_action = Qt.QAction('Reset', self)
        self.reset_input_mesh_action.triggered.connect(self.load_mesh)
        meshMenu.addAction(self.reset_input_mesh_action)
        exit_action = Qt.QAction('Exit', self)
        exit_action.setShortcut('Ctrl+Q')
        exit_action.triggered.connect(self.close)
        self.menuFile.addAction(exit_action)

        # spinBox and slider setting
        self.doubleSpinBox_curvature.valueChanged.connect(self.update_curvature_threshold)
        self.horizontalSlider_curvature.valueChanged.connect(self.update_curvature_threshold)
        self.doubleSpinBox_color.valueChanged.connect(self.update_color_threshold)
        self.horizontalSlider_color.valueChanged.connect(self.update_color_threshold)

        # buttons setting
        self.pushButton_clip_plane.clicked.connect(self.clip_plane_widget)
        self.clip_plane_is_ON = False
        self.pushButton_keep_largest.clicked.connect(self.keep_largest)
        self.pushButton_cell_picking.clicked.connect(self.cell_picking_event)
        self.pushButton_project_to_plane.clicked.connect(self.project_to_best_fit_plane)
        self.pushButton_polar_fitting.clicked.connect(self.polar_fit)
        self.pushButton_generate_toolpath.clicked.connect(self.generate_toolpath)
        self.pushButton_save_contour.clicked.connect(self.save_contour_csv)
        self.pushButton_load_contour.clicked.connect(self.load_contour_csv_event)
        self.pushButton_load_trimesh.clicked.connect(self.load_trimesh)
        self.pushButton_save_toolpath.clicked.connect(self.save_toolpath)
        
        # statusbar setting
        self.pushButton_implant_mesh = Qt.QPushButton("Load Implant")
        self.label_no_implant_mesh_loaded = Qt.QLabel("No implant has been loaded")
        self.pushButton_implant_mesh.clicked.connect(self.load_implant_mesh_event)
        self.statusbar.addWidget(self.pushButton_implant_mesh)
        self.statusbar.addWidget(self.label_no_implant_mesh_loaded)

        self.setWindowTitle("Generate toolpath from defect contour")

        # Enable dragging and dropping onto the GUI
        self.setAcceptDrops(True)

        # Pyvista initialization
        self.plotter.show_axes()
        self.plotter.remove_scalar_bar()
        self.fname = None
        self.mesh = None
        self.mesh_rgb = True
        self.mesh_extracted = None
        self.mesh_implant = None
        self.cell_picking_enabled = False
        self.projection_plane = None
        self.spline_widget = None

        # # debug: load mesh directly
        # self.fname = "../data/3-1 Defect_registered.stl"
        # self.load_mesh()

        # # debug: load contour directly
        # self.fname = "../data/test.csv"
        # self.load_contour_csv(self.fname)
        # fname = "../data/3-1 Defect_registered.stl"
        # self.mesh_defect = pv.read(fname)
        # self.mesh_defect_trimesh = trimesh.load(fname)

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
            if fname.split('/')[-1].split('.')[-1] == 'csv':
                self.load_contour_csv(fname)
            else:
                self.fname = fname
                self.load_mesh()
        else:
            e.ignore()

    def load_trimesh(self):
        self.fname, _ = Qt.QFileDialog.getOpenFileName(self, 'Open file','',"(*.stl) ;; (*.ply)")
        if self.fname != "":
            self.mesh_defect_trimesh = trimesh.load(self.fname)
            self.mesh = pv.read(self.fname)

    def load_mesh_event(self):
        """
        Open a mesh file
        """
        self.fname, _ = Qt.QFileDialog.getOpenFileName(self, 'Open file','',"(*.ply) ;; (*.stl)")
        if self.fname != "":
            self.load_mesh()
            self.mesh_defect_trimesh = trimesh.load(self.fname)

    def load_mesh(self):
        self.mesh = pv.read(self.fname)
        self.setup_curvature_range()
        self.setup_color_range()
        self.plotter.clear()
        self.plotter.add_mesh(self.mesh, rgb=self.mesh_rgb, show_scalar_bar=False, name='mesh')
        self.plotter.reset_camera()
        
    def load_implant_mesh_event(self):
        if self.fname is not None:         
            fname, _ = Qt.QFileDialog.getOpenFileName(self, 'Open file', self.fname.strip(self.fname.split('/')[-1]),"(*.stl) ;; (*.ply)")

            self.checkbox_show_defect = Qt.QCheckBox(self.fname.split('/')[-1])
            self.checkbox_show_defect.stateChanged.connect(self.show_defect)
            self.statusbar.addWidget(self.checkbox_show_defect)

            self.slider_opacity_defect = Qt.QSlider ()
            self.slider_opacity_defect.setMaximum(100)
            self.slider_opacity_defect.setMinimum(0)
            self.slider_opacity_defect.setValue(100)
            self.slider_opacity_defect.valueChanged.connect(self.show_defect)
            self.statusbar.addWidget(self.slider_opacity_defect)
        else:
            fname, _ = Qt.QFileDialog.getOpenFileName(self, 'Open file', '',"(*.stl) ;; (*.ply)")
        if fname != "":
            self.mesh_implant = pv.read(fname)
            
            label_visibility = Qt.QLabel(" Show ")
            self.checkbox_show_implant = Qt.QCheckBox(fname.split('/')[-1])
            self.checkbox_show_implant.stateChanged.connect(self.show_impant)
            self.statusbar.removeWidget(self.pushButton_implant_mesh)
            self.statusbar.removeWidget(self.label_no_implant_mesh_loaded)
            self.statusbar.addWidget(label_visibility)
            self.statusbar.addWidget(self.checkbox_show_implant)
            
            self.slider_opacity_implant = Qt.QSlider ()
            self.slider_opacity_implant.setMaximum(100)
            self.slider_opacity_implant.setMinimum(0)
            self.slider_opacity_implant.setValue(100)
            self.slider_opacity_implant.valueChanged.connect(self.show_impant)
            self.statusbar.addWidget(self.slider_opacity_implant)
        
    def load_contour_csv_event(self):
        """
        Open a contour csv file
        """
        file_dir = ''
        if self.fname is not None:
            file_dir = self.fname.strip(self.fname.split('/')[-1])
        fname, _ = Qt.QFileDialog.getOpenFileName(self, 'Open CSV file', file_dir,"(*.csv) ;; (*.txt)")
        if fname != '':
            self.load_contour_csv(fname)
        
    def load_contour_csv(self, filename):
        self.spline_curve_fit = pv.PolyData(np.genfromtxt(filename)[:,:3])
        self.project_vertices_to_plane(self.spline_curve_fit.points) # generate a best-fit plane
        self.plotter.add_lines(self.spline_curve_fit.points, color='red', name='contour')
        self.plotter.reset_camera()
        
    def show_defect(self, value=None):
        if self.checkbox_show_defect.isChecked():
            self.plotter.add_mesh(self.mesh, rgb=self.mesh_rgb, opacity=self.slider_opacity_defect.value()/100, reset_camera=False, show_scalar_bar=False, name='defect')
        else:
            self.plotter.remove_actor('defect')
        
    def show_impant(self, value=None):
        if self.checkbox_show_implant.isChecked():
            self.plotter.add_mesh(self.mesh_implant, rgb=self.mesh_rgb, opacity=self.slider_opacity_implant.value()/100, reset_camera=False, show_scalar_bar=False, name='implant')
        else:
            self.plotter.remove_actor('implant')
        
    def save_mesh(self):
        """
        Save mesh
        """
        f_name, f_filter = Qt.QFileDialog.getSaveFileName(self, 'Save file', self.fname, "(*.ply) ;; (*.stl)")
        pv.save_meshio(f_name, self.mesh, file_format=f_filter.strip('(*.)'))
    
    def save_contour_csv(self):
        f_name, f_extension = Qt.QFileDialog.getSaveFileName(self, 'Save contour fit curve', self.fname.strip(self.fname.split('/')[-1]), "(*.csv) ;; (*.txt)")
        f_extension = f_extension.strip('(*.)')
        if f_extension == 'csv':
            np.savetxt(f_name, self.spline_curve_fit.points)
        else:
            print("Save as \.txt has not been implemented")   
        
    def save_toolpath(self):
        f_name, f_extension = Qt.QFileDialog.getSaveFileName(self, 'Save generated toolpath', self.fname.strip(self.fname.split('/')[-1]), "(*.csv) ;; (*.txt)")
        f_extension = f_extension.strip('(*.)')
        if f_extension == 'csv':
            np.savetxt(f_name, self.toolpath)
        else:
            print("Save as \.txt has not been implemented")   
        
    def setup_curvature_range(self):
        self.mesh['curvature'] = self.mesh.curvature() # by default mean curvature
        if np.count_nonzero(self.mesh['curvature']) == 0:
            self.doubleSpinBox_curvature.setEnabled(False)
            self.horizontalSlider_curvature.setEnabled(False)
        else:
            max_curvature = max(self.mesh['curvature'])
            min_curvature = min(self.mesh['curvature'])
            self.doubleSpinBox_curvature.setRange(min_curvature, max_curvature)
            self.doubleSpinBox_curvature.setSingleStep(0.01)
            self.horizontalSlider_curvature.setMaximum(int(max_curvature*10000))
            self.horizontalSlider_curvature.setMinimum(int(min_curvature*10000))
            self.horizontalSlider_curvature.setSingleStep(10)
        
    def setup_color_range(self):
        if self.mesh['RGB'] is not None:
            self.mesh['RGB-normed'] = np.array([np.linalg.norm(rgb) for rgb in self.mesh['RGB']])
        elif self.mesh['RGBA'] is not None:
            self.mesh['RGB-normed'] = np.array([np.linalg.norm(rgba[:3]) for rgba in self.mesh['RGBA']])
        else:
            self.mesh_rgb = False
            self.doubleSpinBox_color.setEnabled(False)
            self.horizontalSlider_color.setEnabled(False)
            return
        max_intensity = max(self.mesh['RGB-normed'])
        min_intensity = min(self.mesh['RGB-normed'])
        self.doubleSpinBox_color.setRange(min_intensity, max_intensity)
        self.doubleSpinBox_color.setSingleStep(10)
        self.horizontalSlider_color.setMaximum(int(max_intensity*100))
        self.horizontalSlider_color.setMinimum(int(min_intensity*100))
        self.horizontalSlider_color.setSingleStep(10)
        
    def update_curvature_threshold(self, value):
        if isinstance(value, int):
            self.doubleSpinBox_curvature.setValue(float(value)/10000)
        else:
            self.horizontalSlider_curvature.setValue(int(value*10000))
        # extract points by color followed by curvature
        if self.mesh_rgb:
            self.mesh_extracted = self.mesh.extract_points(np.where(self.mesh['RGB-normed'] > self.doubleSpinBox_color.value()))
            self.mesh_extracted = self.mesh_extracted.extract_points(self.mesh_extracted['curvature'] > self.doubleSpinBox_curvature.value())
        else:
            self.mesh_extracted = self.mesh.extract_points(self.mesh['curvature'] > self.doubleSpinBox_curvature.value())
        self.plotter.add_mesh(self.mesh_extracted, rgb=self.mesh_rgb, show_scalar_bar=False, name='mesh')
        if len(self.mesh_extracted.points) > 0:
            self.plotter.add_mesh(self.mesh_extracted, rgb=self.mesh_rgb, show_scalar_bar=False, name='mesh')
        else:
            self.plotter.remove_actor('mesh')

    def update_color_threshold(self, value):
        if isinstance(value, int):
            self.doubleSpinBox_color.setValue(float(value)/100)
        else:
            self.horizontalSlider_color.setValue(int(value*100))
        # extract points by curvature followed by color intensity
        self.mesh_extracted = self.mesh.extract_points(self.mesh['curvature'] > self.doubleSpinBox_curvature.value())
        self.mesh_extracted = self.mesh_extracted.extract_points(np.where(self.mesh_extracted['RGB-normed'] > self.doubleSpinBox_color.value()))
        if len(self.mesh_extracted.points) > 0:
            self.plotter.add_mesh(self.mesh_extracted, rgb=self.mesh_rgb, show_scalar_bar=False, name='mesh')
        else:
            self.plotter.remove_actor('mesh')
        
    def keep_largest(self):
        self.mesh_extracted.extract_largest(inplace=True)
        self.plotter.add_mesh(self.mesh_extracted, rgb=self.mesh_rgb, show_scalar_bar=False, name='mesh')
        
    def cell_picking_event(self):
        if not self.cell_picking_enabled:
            self.cell_picking_enabled = True
            self.cell_ids_picked = np.array([])
            self.plotter.enable_cell_picking(mesh=self.mesh_extracted, callback=self.cell_picking_callback, show_message=False, style='points', color='red')
            print("Press “r” to enable retangle based selection. Press “r” again to turn it off.\nPress SHIFT to add; CTRL to subtract.")
        else:
            self.delete_selected_cells()
                                             
    def cell_picking_callback(self, cells):
        modifiers = Qt.QApplication.keyboardModifiers()
        if modifiers == QtCore.Qt.ShiftModifier:
            self.cell_ids_picked = np.union1d(self.cell_ids_picked, cells['orig_extract_id'])
        elif modifiers == QtCore.Qt.ControlModifier:
            cell_id_intersection = np.intersect1d(self.cell_ids_picked, cells['orig_extract_id'])
            self.cell_ids_picked = np.setdiff1d(self.cell_ids_picked, cell_id_intersection)
        else:
            self.cell_ids_picked = cells['orig_extract_id']
        mesh_cells_selected = self.mesh_extracted.extract_cells(self.cell_ids_picked)
        self.plotter.add_mesh(mesh_cells_selected, style='points', color='red', name='_cell_picking_selection', pickable=False, reset_camera=False)
        
    def delete_selected_cells(self):
        self.mesh_extracted.remove_cells(self.cell_ids_picked)
        self.plotter.remove_actor('_cell_picking_selection')
        self.plotter.add_mesh(self.mesh_extracted, rgb=self.mesh_rgb, show_scalar_bar=False, name='mesh')

    def clip_plane_widget(self):
        if self.clip_plane_is_ON:
            self.clip_plane_is_ON = False
            self.mesh_extracted = self.plotter.plane_clipped_meshes[0] # assume only a part is left after cropping therefore take the first one
            self.plotter.clear_plane_widgets()
        else:
            self.clip_plane_is_ON = True
            self.plotter.clear()
            self.plotter.add_mesh_clip_plane(self.mesh_extracted, normal='z', rgb=self.mesh_rgb, show_scalar_bar=False, name='mesh')

    def project_vertices_to_plane(self, points):
        # visualize projection plane
        plane, center, normal = pv.fit_plane_to_points(points, return_meta=True)
        # offset the plane along its normal
        offset = self.spinBox_plane_offset.value()
        center = center + offset*normal
        self.projection_plane = pv.Plane(center, normal, 100, 100, 1, 1)
        self.projection_plane.points_projected = pv.PolyData(points).project_points_to_plane(origin=center, normal=normal)
        self.projection_plane.plane_center = center
        self.projection_plane.plane_normal = normal
        if self.checkBox_visulize_on_plane.isChecked():
            self.plotter.add_mesh(self.projection_plane, color='White', opacity=0.1)
            self.plotter.add_mesh(self.projection_plane.points_projected, color='Red', point_size=2.0)

    def project_to_best_fit_plane(self):
        self.project_vertices_to_plane(self.mesh_extracted.points)

    def polar_fit(self):
        if self.projection_plane is None:
            self.project_vertices_to_plane(self.mesh_extracted.points)
        
        # 1. define local 2D coordinate system and convert to polar coordinate system
        vectors = self.projection_plane.points_projected.points - self.projection_plane.points_projected.center
        x_axis = vectors[0] / np.linalg.norm(vectors[0])
        y_axis = np.cross(self.projection_plane.plane_normal, x_axis)
        self.projection_plane.axes = np.column_stack((x_axis.T, y_axis.T, self.projection_plane.plane_normal.T))
        if not self.checkBox_visulize_on_plane.isChecked():
            vectors = self.mesh_extracted.points - self.mesh_extracted.center
        vectors_transformed = np.matmul(vectors, self.projection_plane.axes)
        points_polar = self.cartesian_to_polar(vectors_transformed)
        
        # 2. curve fit using polynomial
        from scipy.optimize import curve_fit
        func = self.poly14
        ans_theta, cov = curve_fit(func, points_polar[:,0], points_polar[:,1])
        ans_h, cov = curve_fit(func, points_polar[:,0], points_polar[:,2])
        if self.checkBox_visulize_on_plane.isChecked():
            num = 200
        else:
            num = self.spinBox_spline_n_handles.value()
        angles_uniform = np.linspace(-np.pi, np.pi, num, endpoint=False)
        radius_uniform = func(angles_uniform, *ans_theta)
        h_uniform = func(angles_uniform, *ans_h)
        curve_parameterized = np.column_stack((angles_uniform, radius_uniform, h_uniform))

        if self.checkBox_visulize_on_plane.isChecked():
            center = self.projection_plane.points_projected.center
            wrap_factor = 20 # wrap the data around to make it looks smooth
            points_polar = points_polar[np.argsort(points_polar[:,0])] # sort indep variable (angles), then copy first and last few elements to wrap around
            points_polar = np.insert(points_polar, 0, points_polar[-wrap_factor:,:] - np.array([2*np.pi,0,0]), axis=0)
            points_polar = np.append(points_polar, points_polar[wrap_factor:2*wrap_factor,:].reshape(wrap_factor,-1) + np.array([2*np.pi,0,0]), axis=0)
            points_curve_fit = self.polar_to_cartesian(curve_parameterized, center, self.projection_plane.axes)
            points_curve_fit = np.append(points_curve_fit, points_curve_fit[0].reshape(1,-1), axis=0) # make it closed
            self.plotter.add_lines(points_curve_fit, color='blue', name='curve_fit_on_projection_plane')
        else:
            center = self.mesh_extracted.center
            self.curve_points = self.polar_to_cartesian(curve_parameterized, center, self.projection_plane.axes)
            # 3. convert the fit curve points into a spline widget
            if self.spline_widget is not None:
                self.plotter.clear_spline_widgets()
                self.spline_widget = None
            self.plotter.add_spline_widget(self.spline_widget_callback, n_handles=len(self.curve_points), resolution=self.spinBox_spline_resolution.value(), pass_widget=True)
        
        # remove mesh extracted
        self.plotter.remove_actor('mesh')
        
    def spline_widget_callback(self, spline_polydata, widget):
        self.spline_curve_fit = spline_polydata
        n_handles = widget.GetNumberOfHandles()
        if self.spline_widget is None:
            self.parametric_spline = vtk.vtkParametricSpline()
            widget.SetParametricSpline(self.parametric_spline)
            self.parametric_spline.SetPoints(pv.vtk_points(self.curve_points))
            self.parametric_spline.SetClosed(True)
            for i in range(n_handles):
                widget.SetHandlePosition(i, *self.curve_points[i])
            self.spline_widget = widget
        else:
            # ensure handle points on mesh surface is snap is turned ON
            if self.checkBox_snap_on.isChecked():
                for i in range(n_handles):
                    if self.mesh_implant is None:
                        mesh = self.mesh
                    else:
                        mesh = self.mesh_implant
                    handle_position = mesh.points[mesh.find_closest_point(widget.GetHandlePosition(i))]
                    widget.SetHandlePosition(i, *handle_position)

    def generate_toolpath(self):
        print("Generating toolpath")
        # d is the diameter of the cutter tool; alpha is the tool tilt-in angle
        d = self.doubleSpinBox_tool_diameter.value()
        r = d/2
        alpha = self.doubleSpinBox_tilt_angle.value() * np.pi / 180
        
        ## vector part ## - we need another set of "shrinked" vectors to generate cut vector
        # 1. generate a series of vectors pointing outward from the center
        vectors = self.spline_curve_fit.points - self.spline_curve_fit.center
        # 2. distribute the vectors along plane normal and tangential directions
        vectors_along_plane_noraml = []
        for vector in vectors:
            vectors_along_plane_noraml.append(np.dot(vector, self.projection_plane.plane_normal) * self.projection_plane.plane_normal)
        vectors_along_plane_noraml = np.array(vectors_along_plane_noraml)
        vectors_along_plane_tangential = vectors - vectors_along_plane_noraml
        # 3. distribute the vectors along tangential and normal based on the given tilt-in angle
        vectors_new_tangential = []
        for vec in vectors_along_plane_tangential:
            vectors_new_tangential.append( vec * (1 - np.sin(alpha)/np.linalg.norm(vec)) )
        vectors_new_tangential = np.array(vectors_new_tangential)
        vectors_new = vectors_new_tangential + vectors_along_plane_noraml

        # **Flip plane normal each time this function is called**
        self.projection_plane.plane_normal = -self.projection_plane.plane_normal
        vectors_new = vectors_new - np.cos(alpha) * self.projection_plane.plane_normal
        vectors_tool = vectors_new - vectors

        # visualize the result
        points_tool = self.spline_curve_fit.points
        # points_seconds = points_tool + vectors_tool*3
        # self.plotter.add_points(points_seconds, style='points', color='Red', point_size=20.0, name='seconds')
        self.toolpath = np.column_stack((points_tool, vectors_tool))
        # self.plotter.add_lines(self.toolpath[:,:3], color='White', name='TCP_pts')
        self.plotter.add_arrows(self.toolpath[:,:3], -self.toolpath[:,3:], mag=3, color='White', name='TCP_axis_neg') # the one used before
        self.plotter.add_arrows(self.toolpath[:,:3], self.toolpath[:,3:], mag=3, color='green', name='TCP_axis_pos')

        # 4. adjust the vectors to fit the defect wall (debug here: use self.mesh_defect)
            # a) find tool_vector_first and tool_vector_second defined in GUI
            # b) determine the direction for ray-casting, by checking if the initial vector is inside/outside the mesh
            # c) perform ray-casting and define the new vector tool (NOTE: take care of the cases without interseciton, ex: bad scan)
        points_vector_tool_first = points_tool + vectors_tool * self.doubleSpinBox_tool_vector_first.value()
        points_vector_tool_second = points_tool + vectors_tool * self.doubleSpinBox_tool_vector_second.value()
        # self.plotter.add_points(points_vector_tool_first, style='points', color='Red', point_size=20.0, name='first')
        # self.plotter.add_points(points_vector_tool_second, style='points', color='Green', point_size=20.0, name='seconds')

        if self.mesh_defect_trimesh is not None:
            # NOTE: the returned idr is the array of ray indices for intersects_location()
            locations_first, idr_first, idt_first = self.mesh_defect_trimesh.ray.intersects_location(points_vector_tool_first, -vectors_along_plane_tangential, multiple_hits=False)
            locations_second, idr_second, idt_second = self.mesh_defect_trimesh.ray.intersects_location(points_vector_tool_second, -vectors_along_plane_tangential, multiple_hits=False)

            # if len(locations_first) > 0:
            #     self.plotter.add_points(locations_first, style='points', color='black', point_size=20.0, name='ray_casted_first')
            # if len(locations_second) > 0:
            #     self.plotter.add_points(locations_second, style='points', color='black', point_size=20.0, name='ray_casted_second')

            # Update vectors_new with valid vectors from ray-casting and get indices for invalid ones
            invalid_ids = None

            if len(locations_first) < len(vectors_tool)//2 or len(locations_second) < len(vectors_tool)//2:
                print("Ray casting fails! Num of Intersection is less than half of the number of the vectors.")
                return

            # check if number of intersections match:
            if len(locations_first) == len(locations_second):
                print(len(locations_first))
                vectors_new = locations_second[np.argsort(idr_second)] - locations_first[np.argsort(idr_first)]
            else:
                print("shape does not match:")
                print(len(locations_first))
                print(len(locations_second))
                # get successful ray casting (ray casting has intersections for both the first point and the second point)
                idr_valid = np.intersect1d(idr_first, idr_second) # ordered
                invalid_ids = np.setdiff1d(np.arange(len(vectors_new)),idr_valid)
                sorter_first = np.argsort(idr_first)
                sorter_second = np.argsort(idr_second)
                idr_first_valid_index = sorter_first[np.searchsorted(idr_first, idr_valid, sorter=sorter_first)]
                idr_second_valid_index = sorter_second[np.searchsorted(idr_second, idr_valid, sorter=sorter_second)]
                vectors_success = locations_second[idr_second_valid_index] - locations_first[idr_first_valid_index]
                # use original vector for unsuccessful ray casting
                vectors_new[idr_valid] = vectors_success

            # self.plotter.add_arrows(self.toolpath[:,:3], vectors_new, mag=3, color='blue', name='TCP_axis_final')

            # find cases that the length of vectors_new is too large
            lengths_vectors_new = np.linalg.norm(vectors_new, axis = 1)
            length_threshold = 2 * (self.doubleSpinBox_tool_vector_second.value() - self.doubleSpinBox_tool_vector_first.value()) # deubg
            if invalid_ids is None:
                invalid_ids = np.where(lengths_vectors_new > length_threshold)[0]
            else:
                invalid_ids = np.union1d(invalid_ids, np.where(lengths_vectors_new > length_threshold)[0])

            # # find cases that ray origin and its intersections are far away
            # lengths_ray_first = locations_first[np.argsort(idr_first)] - points_vector_tool_first
            # lengths_ray_second = locations_second[np.argsort(idr_second)] - points_vector_tool_second
            # incorrect_vector_ids_by_ray_first = np.where(lengths_ray_first > length_threshold)[0]
            # incorrect_vector_ids_by_ray_second = np.where(lengths_ray_second > length_threshold)[0]

            # incorrect_vector_ids = reduce(np.union1d, (incorrect_vector_ids_by_ray_first,\
            #         incorrect_vector_ids_by_vector_second, incorrect_vector_ids_by_vector_second))
            valid_ids = np.setdiff1d(np.arange(len(vectors_new)), invalid_ids)

            lengths_vectors_new = lengths_vectors_new.reshape((-1,1))
            lengths_vectors_new = np.repeat(lengths_vectors_new, 3, axis=1)
            vectors_new = vectors_new / lengths_vectors_new
            vectors_valid = vectors_new[valid_ids]
            vectors_invalid = vectors_new[invalid_ids]

            # visualize vectors 
            self.plotter.add_arrows(self.toolpath[valid_ids , :3], vectors_valid, mag=3, color='blue', name='TCP_axis_valid')
            if len(invalid_ids) > 0:
                self.plotter.add_arrows(self.toolpath[invalid_ids , :3], vectors_invalid, mag=3, color='red', name='TCP_axis_invalid')

            # # closest points
            # closest_pts = []
            # for point in points_seconds:
            #     closest_pt_id = self.mesh_implant.find_closest_point(point)
            #     closest_pt = self.mesh_implant.points[closest_pt_id]
            #     closest_pts.append(closest_pt)
            # closest_pts = np.array(closest_pts)

        # ## point part ## - to consider tool diameter compensation
        # # 1. distribute the tooltip offset displacement along tangential
        # vectors_tangential_offset = []
        # for vec in vectors_along_plane_tangential:
        #     vectors_tangential_offset.append(r * np.cos(alpha) * vec / np.linalg.norm(vec))
        # vectors_tangential_offset = np.array(vectors_tangential_offset)
        # points_tool = self.spline_curve_fit.points + vectors_tangential_offset
        
        # self.toolpath = np.column_stack((points_tool, vectors_tool))
        # self.plotter.add_lines(self.toolpath[:,:3], color='White', name='TCP_pts')
        # self.plotter.add_arrows(self.toolpath[:,:3], -self.toolpath[:,3:], mag=3, color='White', name='TCP_axis')

        print("Finished toolpath generation")

    @staticmethod
    def poly12(x, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12):
        # polynomial of degree = 14
        return  sum([a0, a1*x, a2*x**2, a3*x**3, a4*x**4, a5*x**5, a6*x**6, a7*x**7, a8*x**8, a9*x**9, a10*x**10, a11*x**11, a12*x**12])

    @staticmethod
    def poly14(x, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14):
        # polynomial of degree = 14
        return  sum([a0, a1*x, a2*x**2, a3*x**3, a4*x**4, a5*x**5, a6*x**6, a7*x**7, a8*x**8, a9*x**9, a10*x**10, a11*x**11, a12*x**12, a13*x**13, a14*x**14])

    @staticmethod
    def cartesian_to_polar(points):
        """ Function to transform points in 2D cartesian to 2D polar coordinate with height.
        Args:
            points: 2D points with height in cartesian coordinate system, array of (n,3) 
        Returns:
            points_polarized: points in 2D polar with height
        """
        # transform 2D points to polar system (theta, r)
        x, y, h = points[:,0], points[:,1], points[:,2]
        radius = np.sqrt(x**2 + y**2).reshape((-1,1)) # shape: nx1
        angle = np.arctan2(y,x).reshape((-1,1)) # shape: nx1
        points_polarized = np.column_stack((angle, radius, h))

        return points_polarized

    @staticmethod
    def polar_to_cartesian(points_polar, center, axes):
        """ Function to transform points in 2D polar coordinate to 3D cartesian.
        Args:
            points_polar: 2D points in polar coordinate system, array of (n,2) 
            center: 3D coordinate of the origin of 2D polar coordinate system
            axes: x, y, z axis of local coordinate system, represented in 3D, column array of (3, 3)
            onplane: if True, points are projected on the plane
        Returns:
            points_transformed_3d: points in 3D cartesian
        """
        # convert desired_points back to original coordinate system
        angle = points_polar[:,0] # shape: (n,)
        radius = points_polar[:,1] # shape: (n,)
        h = points_polar[:,2]
        x_2d = radius * np.cos(angle)
        y_2d = radius * np.sin(angle)
        points_local = np.column_stack((x_2d, y_2d, h))
        points_3d = center + np.matmul(points_local, axes.T)
        
        return points_3d

if __name__ == '__main__':
    app = Qt.QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec_())
import os, sys, logging
from typing import Any, Callable, Dict, Generator, List, Optional, Tuple, Union

import pyvista as pv
from pyvista import global_theme
from pyvistaqt import MainWindow, QtInteractor
from pyvistaqt.counter import Counter
from pyvistaqt.dialog import FileDialog, ScaleAxesDialog
from editor import Editor
from pyvistaqt.utils import (
    _check_type,
    _create_menu_bar,
    _setup_application,
    _setup_ipython,
    _setup_off_screen,
)

from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QAction, QFrame, QSplitter, QGestureEvent, QGridLayout, QMenuBar, QToolBar, QHBoxLayout, QWidget


LOG = logging.getLogger("pyvistaqt")
LOG.setLevel(logging.CRITICAL)
LOG.addHandler(logging.StreamHandler())

class BigssPlotter(QtInteractor):
    """BIGSS Qt interactive plotter
    
    My modified Background plotter for pyvista that allows you to maintain an
    interactive plotting window without blocking the main python thread.
    
    Parameters
    ----------
    show :
        Show the plotting window.  If ``False``, show this window by running ``show()``
    app : optional
        Creates a `QApplication` if left as `None`.
    window_size :
        Window size in pixels.  Defaults to ``[1024, 768]``
    off_screen :
        Renders off screen when True.  Useful for automated screenshots or debug testing.
    toolbar : bool
        If True, display the default camera toolbar. Defaults to True.
    menu_bar : bool
        If True, display the default main menu. Defaults to True.
    editor: bool
        If True, display the VTK object editor. Defaults to True.
    update_app_icon :
        If True, update_app_icon will be called automatically to update the
        Qt app icon based on the current rendering output. If None, the
        logo of PyVista will be used. If False, no icon will be set.
        Defaults to None.
    title : str, optional
        Title of plotting window.
    multi_samples : int, optional
        The number of multi-samples used to mitigate aliasing. 4 is a
        good default but 8 will have better results with a potential
        impact on performance.
    line_smoothing : bool, optional
        If True, enable line smothing
    point_smoothing : bool, optional
        If True, enable point smothing
    polygon_smoothing : bool, optional
        If True, enable polygon smothing
    auto_update : float, bool, optional
        Automatic update rate in seconds.  Useful for automatically
        updating the render window when actors are change without
        being automatically ``Modified``.  If set to ``True``, update
        rate will be 1 second.
    Examples
    --------
    >>> import pyvista as pv
    >>> from pyvistaqt import BigssPlotter
    >>> plotter = BigssPlotter()
    >>> plotter.add_mesh(pv.Sphere())
    """

    def __init__(
        self,
        show: bool = True,
        app: Optional[QApplication] = None,
        window_size: Optional[Tuple[int, int]] = None,
        off_screen: Optional[bool] = None,
        toolbar: bool = True,
        menu_bar: bool = True,
        editor: bool = True,
        update_app_icon: Optional[bool] = None,
        **kwargs: Any
        ) -> None:
        """Initialize the qt plotter."""
        self._closed = True  # avoid recursion of the close() function by setting self._closed=True until the BasePlotter.__init__ is called
        LOG.debug("BigssPlotter init start")
        _check_type(show, "show", [bool])
        _check_type(app, "app", [QApplication, type(None)])
        _check_type(window_size, "window_size", [tuple, type(None)])
        _check_type(off_screen, "off_screen", [bool, type(None)])
        _check_type(toolbar, "toolbar", [bool])
        _check_type(menu_bar, "menu_bar", [bool])
        _check_type(editor, "editor", [bool])
        _check_type(update_app_icon, "update_app_icon", [bool, type(None)])

        # toolbar
        self._view_action: QAction = None
        self.default_camera_tool_bar: QToolBar = None
        self.saved_camera_positions: Optional[list] = None
        self.saved_cameras_tool_bar: QToolBar = None

        # menu bar
        self.main_menu: QMenuBar = None
        self._edl_action: QAction = None
        self._menu_close_action: QAction = None
        self._parallel_projection_action: QAction = None
        
        # editor
        self.editor: Optional[Editor] = None

        self.active = True
        self.counters: List[Counter] = []

        self.app = _setup_application(app)
        self.off_screen = _setup_off_screen(off_screen)

        self.app_window = MainWindow(title=kwargs.get("title", global_theme.title))
        self.splitter = QSplitter(parent=self.app_window)
        self.frame = QFrame(parent=self.splitter)
        self.frame.setFrameStyle(QFrame.NoFrame)
        super().__init__(parent=self.frame, off_screen=off_screen, **kwargs)
        assert not self._closed
        hlayout = QHBoxLayout()
        hlayout.addWidget(self)
        self.frame.setLayout(hlayout)
        self.splitter.addWidget(self.frame)
        if editor:
            self.add_editor()
            self.splitter.addWidget(self.editor)
            self.splitter.setStretchFactor(0, 2)
            self.splitter.setStretchFactor(1, 1)
        self.app_window.setCentralWidget(self.splitter)
        self.app_window.grabGesture(QtCore.Qt.PinchGesture)
        self.app_window.signal_gesture.connect(self.gesture_event)
        self.app_window.signal_close.connect(self._close)

        if menu_bar:
            self.add_menu_bar()
        if toolbar:
            self.add_toolbars()

        if show and not self.off_screen:  # pragma: no cover
            self.app_window.show()

    LOG.debug("BiggsPlotter initialized")

    def _add_mesh(self, show: bool = True) -> FileDialog:
        def add_mesh_from_file(filename):
            self.add_mesh(pv.read(filename), rgb=True)
            self.editor.update()

        return FileDialog(
            self.app_window,
            filefilter=["PLY (*.ply)", "STL (*.stl)"],
            save_mode=False,
            show=show,
            callback=add_mesh_from_file,
        )

    def _qt_screenshot(self, show: bool = True) -> FileDialog:
        return FileDialog(
            self.app_window,
            filefilter=["Image File (*.png)", "JPEG (*.jpeg)"],
            show=show,
            directory=bool(os.getcwd()),
            callback=self.screenshot,
        )

    def _toggle_edl(self) -> None:
        if hasattr(self.renderer, "edl_pass"):
            return self.renderer.disable_eye_dome_lighting()
        return self.renderer.enable_eye_dome_lighting()

    def _toggle_parallel_projection(self) -> None:
        if self.camera.GetParallelProjection():
            return self.disable_parallel_projection()
        return self.enable_parallel_projection()

    def add_callback(
        self, func: Callable, interval: int = 1000, count: Optional[int] = None
    ) -> None:
        """Add a function that can update the scene in the background.
        Parameters
        ----------
        func :
            Function to be called with no arguments.
        interval :
            Time interval between calls to `func` in milliseconds.
        count :
            Number of times `func` will be called. If None,
            `func` will be called until the main window is closed.
        """
        self._callback_timer = QtCore.QTimer(parent=self.app_window)
        self._callback_timer.timeout.connect(func)
        self._callback_timer.start(interval)
        self.app_window.signal_close.connect(self._callback_timer.stop)
        if count is not None:
            counter = Counter(count)
            counter.signal_finished.connect(self._callback_timer.stop)
            self._callback_timer.timeout.connect(counter.decrease)
            self.counters.append(counter)

    def save_camera_position(self) -> None:
        """Save camera position to saved camera menu for recall."""
        if self.saved_camera_positions is not None:
            # pylint: disable=attribute-defined-outside-init
            self.camera_position: Any
            self.saved_camera_positions.append(self.camera_position)
            ncam = len(self.saved_camera_positions)
        if self.camera_position is not None:
            camera_position: Any = self.camera_position[:]  # py2.7 copy compatibility

        if hasattr(self, "saved_cameras_tool_bar"):

            def load_camera_position() -> None:
                # pylint: disable=attribute-defined-outside-init
                self.camera_position = camera_position

            self.saved_cameras_tool_bar.addAction(
                "Cam %2d" % ncam, load_camera_position
            )
            if ncam < 10:
                self.add_key_event(str(ncam), load_camera_position)

    def clear_camera_positions(self) -> None:
        """Clear all camera positions."""
        if hasattr(self, "saved_cameras_tool_bar"):
            for action in self.saved_cameras_tool_bar.actions():
                if action.text() not in ["Save Camera", "Clear Cameras"]:
                    self.saved_cameras_tool_bar.removeAction(action)
        self.saved_camera_positions = []

    def _add_action(self, tool_bar: QToolBar, key: str, method: Any) -> QAction:
        action = QAction(key, self.app_window)
        action.triggered.connect(method)
        tool_bar.addAction(action)
        return action

    def add_toolbars(self) -> None:
        """Add the toolbars."""
        # Camera toolbar
        self.default_camera_tool_bar = self.app_window.addToolBar("Camera Position")

        def _view_vector(*args: Any) -> None:
            return self.view_vector(*args)

        cvec_setters = {
            # Viewing vector then view up vector
            "Top (-Z)": lambda: _view_vector((0, 0, 1), (0, 1, 0)),
            "Bottom (+Z)": lambda: _view_vector((0, 0, -1), (0, 1, 0)),
            "Front (-Y)": lambda: _view_vector((0, 1, 0), (0, 0, 1)),
            "Back (+Y)": lambda: _view_vector((0, -1, 0), (0, 0, 1)),
            "Left (-X)": lambda: _view_vector((1, 0, 0), (0, 0, 1)),
            "Right (+X)": lambda: _view_vector((-1, 0, 0), (0, 0, 1)),
            "Isometric": lambda: _view_vector((1, 1, 1), (0, 0, 1)),
        }
        for key, method in cvec_setters.items():
            self._view_action = self._add_action(
                self.default_camera_tool_bar, key, method
            )
        # pylint: disable=unnecessary-lambda
        self._add_action(
            self.default_camera_tool_bar, "Reset", lambda: self.reset_camera()
        )

        # Saved camera locations toolbar
        self.saved_camera_positions = []
        self.saved_cameras_tool_bar = self.app_window.addToolBar(
            "Saved Camera Positions"
        )

        self._add_action(
            self.saved_cameras_tool_bar, "Save Camera", self.save_camera_position
        )
        self._add_action(
            self.saved_cameras_tool_bar, "Clear Cameras", self.clear_camera_positions
        )

    def add_menu_bar(self) -> None:
        """Add the main menu bar."""
        self.main_menu = _create_menu_bar(parent=self.app_window)
        self.app_window.signal_close.connect(self.main_menu.clear)

        file_menu = self.main_menu.addMenu("File")
        file_menu.addAction("Add Mesh", self._add_mesh)
        file_menu.addAction("Take Screenshot", self._qt_screenshot)
        file_menu.addSeparator()
        # member variable for testing only
        self._menu_close_action = file_menu.addAction("Exit", self.app_window.close)

        view_menu = self.main_menu.addMenu("View")
        self._edl_action = view_menu.addAction(
            "Toggle Eye Dome Lighting", self._toggle_edl
        )
        view_menu.addAction("Clear All", self.clear)

        cam_menu = view_menu.addMenu("Camera")
        self._parallel_projection_action = cam_menu.addAction(
            "Toggle Parallel Projection", self._toggle_parallel_projection
        )

        tool_menu = self.main_menu.addMenu("Tools")
        picking_menu = tool_menu.addMenu("Picking")
        picking_menu.addAction("Enable Point Picking", self.enable_point_picking)
        picking_menu.addAction("Enable Cell Picking (through)", self.enable_cell_picking)
        picking_menu.addAction(
            "Enable Cell Picking (visible)",
            lambda: self.enable_cell_picking(through=False),
        )

        view_menu.addSeparator()
        # Orientation marker
        orien_marker_visilibity = view_menu.addAction("Show Axes")
        orien_marker_visilibity.setCheckable(True)
        orien_marker_visilibity.toggled.connect(self.set_show_axes)

        # Bounds axes
        axes_menu = view_menu.addMenu("Bounds Axes")
        axes_menu.addAction("Add Bounds Axes (front)", self.show_bounds)
        axes_menu.addAction("Add Bounds Grid (back)", self.show_grid)
        axes_menu.addAction("Add Bounding Box", self.add_bounding_box)
        axes_menu.addSeparator()
        axes_menu.addAction("Remove Bounding Box", self.remove_bounding_box)
        axes_menu.addAction("Remove Bounds", self.remove_bounds_axes)

        # A final separator to separate OS options
        view_menu.addSeparator()

    def set_show_axes(self, state : bool):
        if state:
            self.renderer.show_axes()
        else:
            self.renderer.hide_axes()

    def clear(self):
        """Override the clear function"""
        super().clear()
        self.editor.update()

    def add_editor(self) -> Editor:
        """Add the editor."""
        self.editor = Editor(parent=self.app_window, renderers=self.renderers)
        self.app_window.signal_close.connect(self.editor.close)

    def close(self) -> None:
        """Close the plotter.
        This function closes the window which in turn will
        close the plotter through `signal_close`.
        """
        if not self._closed:
            # Can get:
            #
            #     RuntimeError: wrapped C/C++ object of type MainWindow has
            #     been deleted
            #
            # So let's be safe and try/except this in case of a problem.
            try:
                self.app_window.close()
            except Exception:
                pass

    def _close(self) -> None:
        super().close()

    

if __name__ == "__main__":
    app = QApplication(sys.argv)
    plotter = BigssPlotter()
    plotter.add_mesh(pv.Sphere())
    plotter.editor.update()
    sys.exit(app.exec_())

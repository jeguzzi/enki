from __future__ import annotations

import sys
from collections.abc import Callable
from typing import TYPE_CHECKING, SupportsFloat

if sys.version_info >= (3, 11):
    from typing import Unpack
else:
    from typing_extensions import Unpack

import numpy as np
from PySide6.QtCore import QSize, Qt, Slot
from PySide6.QtGui import QImage
from PySide6.QtOpenGLWidgets import QOpenGLWidget

from .. import Image, World
from .camera import HasCamera
from .renderer import Renderer
from .types import CameraConfig, Pixel, Vector3
from .ui import UI
from .utils import get_position_of_pixel, init, run_loop

if TYPE_CHECKING:
    from PySide6.QtCore import QEvent
    from PySide6.QtGui import QHideEvent, QMouseEvent, QWheelEvent
    from PySide6.QtWidgets import QWidget


class WorldView(QOpenGLWidget, HasCamera):
    """
    A QOpenGLWidget that displays a world.

    Basic example::

        >>> import pyenki.viewer
        >>> # setup a world
        >>> world = ...
        >>> # setup Qt: needs to be called before creating the first view
        >>> pyenki.viewer.init()
        >>> viewer = pyenki.WorldView(world=world)
        >>> viewer.show()
        >>> viewer.start_updating_world(0.1)
        >>> # executes the Qt runloop for a while
        >>> pyenki.viewer.run(duration=10)

    Example of composition of two views of the same world::

        >>> import pyenki.viewer
        >>> from PySide6.QtWidgets import QHBoxLayout, QWidget
        >>> # setup a world
        >>> world = ...
        >>> viewer_1 = pyenki.WorldView(
                world=world, camera_position=(-20, -20), camera_altitude=20)
        >>> viewer_1.point_camera(target_position=(0, 0), target_altitude=5)
        >>> viewer_2 = pyenki.WorldView(
                world=world, helpers=False, camera_is_ortho=True, camera_altitude=30)
        >>> window = QWidget()
        >>> hbox = QHBoxLayout(window)
        >>> window.resize(960, 320)
        >>> hbox.addWidget(viewer_1.widget)
        >>> hbox.addWidget(viewer_2.widget)
        >>> window.show()
        >>> viewer_1.start_updating_world(0.1)
        >>> pyenki.viewer.run(duration=10)

    Attributes:
        world (World | None): the world to display.
        walls_height (float): the height of the world boundary in cm (readonly).
        helpers (bool): whether to display the helpers widgets.
        image (Image): the currently rendered image (readonly).
        qt_widget (QOpenGLWidget): a PyQt-compatible widget (readonly).
        pyside_widget (QOpenGLWidget): a PySide-compatible widget (readonly).
    """

    def __init__(self,
                 parent: QWidget | None = None,
                 /,
                 world: World | None = None,
                 fps: SupportsFloat = 30.0,
                 update_world: bool = False,
                 time_step: SupportsFloat = 0.0,
                 factor: SupportsFloat = 1.0,
                 physics_oversampling: int = 3,
                 callback: Callable[[World], None] | None = None,
                 helpers: bool = True,
                 walls_height: SupportsFloat = 10.0,
                 **camera_config: Unpack[CameraConfig]) -> None:
        """
        Constructs a new instance.

        Args:
            world (World | None): The world to display.
            fps (float): The framerate of the viewer in frames per second.
            update_world (bool): Whether to trigger world updates before redrawing.
            time_step (float): The simulation time step in seconds.
            factor (bool): The real-time factor. If larger than one, the simulation
                           will run faster then real-time.
            physics_oversampling (int):  The number of times the physics is updated per step
                                         to get a more fine-grained physical simulation
                                         compared to the sensor-motor loop.
            callback (Callable[[World], None] | None): An optional callback executed at each simulation step.
            helpers (bool): Whether to display the helpers widgets.
            walls_height (float): the height of the world boundary in cm.
            **camera_config (CameraConfig): the camera configuration.
        """
        QOpenGLWidget.__init__(self, parent)
        self._world = world
        HasCamera.__init__(self, world=world, **camera_config)
        self._ui = UI(self)
        self._update_world = update_world
        self._factor = float(factor)
        self.renderer: Renderer | None = None
        self.helpers = helpers
        self._wall_height = float(walls_height)
        self._next_update_time = 0.0
        self._world_time_step = 0.0
        self._timer_period = 0.0
        self._rt_factor = 1.0
        self.physics_oversampling = physics_oversampling
        self.callback = callback
        self.cursor_position: Vector3 | None = None
        fps = float(fps)
        if fps > 0:
            self._timer_period = 1.0 / fps
            self.startTimer(int(self._timer_period * 1e3))
        if update_world:
            self.start_updating_world(time_step, factor)

    def save_image(self, path: str) -> None:
        """
        Saves an image.

        :param      path:  The file path
        """
        fb = self.grabFramebuffer()
        fb.save(path)

    @property
    def image(self) -> Image:
        image = self.grabFramebuffer().convertToFormat(
            QImage.Format.Format_RGB888)
        buffer = image.bits()
        shape = (image.height(), image.width(), 3)
        # buffer.setsize(np.prod(shape))
        return np.frombuffer(buffer, np.uint8).reshape(shape)

    @property
    def walls_height(self) -> float:
        return self._wall_height

    @walls_height.setter
    def walls_height(self, value: float) -> None:
        self._wall_height = max(0, value)

    @property
    def world(self) -> World | None:
        return self._world

    @world.setter
    def world(self, value: World | None) -> None:
        if value != self._world:
            if self._world and self.renderer:
                self.renderer.remove_world(self._world)
            if value and self.renderer:
                self.renderer.add_world(value)
            self._world = value

    def start_updating_world(self,
                             time_step: SupportsFloat = 0.0,
                             factor: SupportsFloat = 1.0) -> None:
        """
        Starts updating the world in real-time.

        :param      time_step:  The world time step
        :param      factor:     The real time factor
        """
        self._update_world = True
        self._rt_factor = float(factor)
        time_step = float(time_step)
        self._world_time_step = time_step if time_step > 0 else self._timer_period
        self._next_update_time = self._world_time_step

    def stop_updating_world(self) -> None:
        """
        Stops updating the world.
        """
        self._update_world = False

    @Slot()
    def cleanup(self) -> None:
        if self.renderer:
            self.makeCurrent()
            self.renderer.context_will_be_destroyed(self.context())
            self.doneCurrent()

    def timerEvent(self, event: QEvent) -> None:
        if self._update_world and self.world:
            self._next_update_time -= self._rt_factor * self._timer_period
            while self._next_update_time < 0:
                self.world.step(self._world_time_step,
                                self.physics_oversampling)
                if self.callback:
                    self.callback(self.world)
                self._next_update_time += self._world_time_step
        self.update()

    def initializeGL(self) -> None:
        self.renderer = Renderer.get(self.context())
        self.renderer.init()

    def paintGL(self) -> None:
        if self.world and self.renderer:
            self.update_camera()
            self.renderer.draw(self.world, self.walls_height,
                               self.camera.matrix, self.camera.projection,
                               self._ui.selected_object)

    def resizeGL(self, width: int, height: int) -> None:
        self._ui.on_resize(width, height)

    def hideEvent(self, event: QHideEvent) -> None:
        self.cleanup()
        super().hideEvent(event)

    @staticmethod
    def get_pixel(event: QMouseEvent) -> Pixel:
        p = event.position()
        return int(p.x()), int(p.y())

    @staticmethod
    def get_buttons(event: QMouseEvent) -> tuple[bool, bool]:
        left_button = (bool(event.buttons() & Qt.MouseButton.LeftButton)
                       or event.button() == Qt.MouseButton.LeftButton)
        right_button = (bool(event.buttons() & Qt.MouseButton.RightButton)
                        or event.button() == Qt.MouseButton.RightButton)
        return left_button, right_button

    def mousePressEvent(self, event: QMouseEvent) -> None:
        self._ui.on_mouse_press(self.get_pixel(event),
                                *self.get_buttons(event))

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        self._ui.on_mouse_double_click()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        self._ui.on_mouse_release(*self.get_buttons(event))

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        shift = bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier)
        self._ui.on_mouse_move(self.get_pixel(event),
                               *self.get_buttons(event),
                               shift=shift)

    def wheelEvent(self, event: QWheelEvent) -> None:
        delta = event.angleDelta().y()
        self._ui.on_wheel(delta)

    def minimumSizeHint(self) -> QSize:
        return QSize(50, 50)

    def sizeHint(self) -> QSize:
        return QSize(400, 400)

    @property
    def pyside_widget(self) -> QOpenGLWidget:
        return self

    @property
    def pyqt_widget(self) -> object:
        raise RuntimeError(
            "Cannot convert PySide6.QOpenGLWidget to PyQt6.QOpenGLWidget")

    def get_position_of_pixel(self, pixel: Pixel) -> Vector3 | None:
        if self.world:
            r = self.devicePixelRatio()
            pixel = pixel[0] / self.width(), pixel[1] / self.height()
            width = int(self.width() * r)
            height = int(self.height() * r)
            self.makeCurrent()
            p = get_position_of_pixel(pixel,
                                      width=width,
                                      height=height,
                                      camera=self.camera)
            self.doneCurrent()
            return p
        return None


def run_in_viewer(world: World,
                  /,
                  fps: SupportsFloat = 30,
                  time_step: SupportsFloat = 0,
                  factor: SupportsFloat = 1,
                  physics_oversampling: int = 3,
                  callback: Callable[[World], None] | None = None,
                  termination: Callable[[World], bool] | None = None,
                  helpers: bool = True,
                  walls_height: SupportsFloat = 10,
                  duration: SupportsFloat = -1,
                  **camera_config: Unpack[CameraConfig]) -> None:
    """
    Runs a simulation while displaying it in real-time in a viewer.

    Args:
        world (World): the world to display and run.
        fps (float): The framerate of the viewer in frames per second.
        time_step (float): The simulation time step in seconds.
        factor (bool): The real-time factor. If larger than one, the simulation
                       will run faster then real-time.
        physics_oversampling (int):  The number of times the physics is updated per step
                                     to get a more fine-grained physical simulation
                                     compared to the sensor-motor loop.
        callback (Callable[[World], None] | None): An optional callback executed at each simulation step.
        termination (Callable[[World], bool] | None): An optional function that makes
            the simulation terminate when it returns True
        helpers (bool): Whether to display the helpers widgets.
        walls_height (float): the height of the world boundary in cm.
        duration (float): duration of the simulation in simulated time.
                          Negative values are interpreted as infinite duration.
        **camera_config (CameraConfig): the camera configuration.
    """
    init()
    loop = run_loop(float(duration) / float(factor))
    if not loop:
        return
    cb = callback
    if termination:
        def cb(world: World) -> None:
            if callback:
                callback(world)
            if termination(world):
                loop.quit()

    viewer = WorldView(world=world,
                       fps=fps,
                       update_world=True,
                       time_step=time_step,
                       factor=factor,
                       physics_oversampling=physics_oversampling,
                       callback=cb,
                       helpers=helpers,
                       walls_height=walls_height,
                       **camera_config)
    viewer.setWindowTitle("PyEnki Viewer")
    viewer.show()
    loop.exec()

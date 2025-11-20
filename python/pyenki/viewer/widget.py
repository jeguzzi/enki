import typing

import numpy as np
from PySide6.QtCore import QEvent, QSize, Qt, Slot
from PySide6.QtGui import QHideEvent, QImage, QMouseEvent, QWheelEvent
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import QWidget

from .. import Image, World
from .camera import HasCamera
from .renderer import Renderer
from .types import CameraConfig, Pixel, Vector3
from .ui import UI
from .utils import get_position_of_pixel, init, run


class WorldView(QOpenGLWidget, HasCamera):

    def __init__(self,
                 parent: QWidget | None = None,
                 /,
                 world: World | None = None,
                 fps: typing.SupportsFloat = 30.0,
                 update_world: bool = False,
                 time_step: typing.SupportsFloat = 0.0,
                 factor: typing.SupportsFloat = 1.0,
                 helpers: bool = True,
                 walls_height: typing.SupportsFloat = 10.0,
                 **camera_config: typing.Unpack[CameraConfig]) -> None:
        QOpenGLWidget.__init__(self, parent)
        self._world = world
        HasCamera.__init__(self, world=world, **camera_config)
        self._ui = UI(self)
        self._update_world = update_world
        self._factor = float(factor)
        self.renderer: Renderer | None = None
        self.helpers = helpers
        self.tracking = False
        self._wall_height = float(walls_height)
        self._next_update_time = 0.0
        self._world_time_step = 0.0
        self._timer_period = 0.0
        self._rt_factor = 1.0
        self.cursor_position: Vector3 | None = None
        fps = float(fps)
        if fps > 0:
            self._timer_period = 1.0 / fps
            self.startTimer(int(self._timer_period * 1e3))
        if update_world:
            self.start_updating_world(time_step, factor)

    def save_image(self, path: str) -> None:
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
    def widget(self) -> typing.Self:
        return self

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
                             time_step: typing.SupportsFloat = 0.0,
                             factor: typing.SupportsFloat = 1.0) -> None:
        self._update_world = True
        self._rt_factor = float(factor)
        time_step = float(time_step)
        self._world_time_step = time_step if time_step > 0 else self._timer_period
        self._next_update_time = self._world_time_step

    def stop_updating_world(self) -> None:
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
                self.world.step(self._world_time_step, 3)
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

    def mousePressEvent(self, event: QMouseEvent) -> None:
        self._ui.on_mouse_press(self.get_pixel(event),
                                left_button=bool(event.buttons()
                                                 & Qt.MouseButton.LeftButton))

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        self._ui.on_mouse_double_click()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        self._ui.on_mouse_release()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        self._ui.on_mouse_move(
            self.get_pixel(event),
            left_button=bool(event.buttons() & Qt.MouseButton.LeftButton),
            right_button=bool(event.buttons() & Qt.MouseButton.RightButton),
            shift=bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier))

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
            pixel = int(pixel[0] * r), int(pixel[1] * r)
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


def run_in_viewer(self: World,
                  /,
                  fps: typing.SupportsFloat = 30,
                  time_step: typing.SupportsFloat = 0,
                  factor: typing.SupportsFloat = 1,
                  helpers: bool = True,
                  walls_height: typing.SupportsFloat = 10,
                  duration: typing.SupportsFloat = -1,
                  **camera_config: typing.Unpack[CameraConfig]) -> None:
    init()
    viewer = WorldView(world=self,
                       fps=fps,
                       update_world=True,
                       time_step=time_step,
                       factor=factor,
                       helpers=helpers,
                       walls_height=walls_height,
                       **camera_config)
    viewer.setWindowTitle("PyEnki Viewer")
    viewer.show()
    run(float(duration) / float(factor))

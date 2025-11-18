import typing

import numpy as np
from PySide6.QtCore import QEvent, QPointF, QSize, Qt, Slot
from PySide6.QtGui import QCursor, QHideEvent, QImage, QMouseEvent, QWheelEvent
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import QWidget

from .. import Image, PhysicalObject, World
from .camera import CameraConfig, HasCamera, Vector3
from .renderer import Renderer
from .utils import get_object_at, get_position_of_pixel, init, run


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
        self._last_pos = QPointF()
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
        self.callback = None
        self.selected_object: PhysicalObject | None = None
        self.pointed_object: PhysicalObject | None = None
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
            self.renderer.draw(self.world, self.walls_height,
                               self.camera.matrix, self.camera.projection,
                               self.selected_object)
            self.update_cursor()
            if self.callback:
                self.callback(self)

    def resizeGL(self, width: int, height: int) -> None:
        # print("resizeGL", width, height)
        self.camera.set_viewport(width, height)

    def hideEvent(self, event: QHideEvent) -> None:
        self.cleanup()
        super().hideEvent(event)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        self._last_pos = event.position()
        if not self.selected_object:
            self.selected_object = self.pointed_object

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        self.selected_object = None

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        pos = event.position()
        dx = pos.x() - self._last_pos.x()
        dy = pos.y() - self._last_pos.y()
        self._last_pos = pos

        if self.selected_object:
            if event.buttons() & Qt.MouseButton.RightButton:
                sensitivity = 10 / (1 + self.width())
                self.selected_object.angle -= sensitivity * dx
                return
            p = self.cursor_position
            if (event.buttons() & Qt.MouseButton.LeftButton) and p is not None:
                self.selected_object.position = p[:2]
                self.selected_object.velocity = (0, 0)
                self.selected_object.angular_speed = 0
                return

        if event.modifiers() & Qt.KeyboardModifier.ShiftModifier:
            sensitivity = -(1 + 0.1 * self.camera_altitude) * 0.1
            self.camera.position += sensitivity * dy * self.camera.forward
        elif event.buttons() & Qt.MouseButton.LeftButton:
            sensibility = 20.0 + 2. * self.camera_altitude
            size_factor = 1.0 + (self.width() + self.height()) / 2
            self.camera.position -= sensibility * (
                dx * self.camera.left + dy * self.camera.up) / size_factor
        else:
            sensitivity = 4.0
            self.camera_yaw -= sensitivity * dx / (1 + self.width())
            delta = 0.01
            self.camera_pitch = np.clip(
                self.camera_pitch - sensitivity * dy / (1 + self.height()),
                -np.pi / 2 + delta, np.pi / 2 - delta)

    def wheelEvent(self, event: QWheelEvent) -> None:
        delta = event.angleDelta().y()
        sensitivity = (1 + 0.1 * self.camera_altitude) * 0.003
        self.camera.position += sensitivity * delta * self.camera.forward
        self.camera.position[2] = max(0, self.camera.position[2])

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

    def get_cursor_position(self) -> Vector3 | None:
        p = self.mapFromGlobal(QCursor.pos())
        if not self.rect().contains(p, True):
            return None
        r = int(self.devicePixelRatio())
        return get_position_of_pixel((r * p.x(), r * p.y()),
                                     width=int(self.width() * r),
                                     height=int(self.height() * r),
                                     camera=self.camera)

    def get_object_at_cursor(self,
                             tolerance: float = 0.2) -> PhysicalObject | None:
        if self.world:
            p = self.get_cursor_position()
            if p is not None:
                return get_object_at(self.world, p[:2], tolerance=tolerance)
        return None

    def update_cursor(self, tolerance: float = 2) -> None:
        self.pointed_object = None
        self.cursor_position = None
        if self.world:
            self.cursor_position = self.get_cursor_position()
            if self.cursor_position is not None:
                self.pointed_object = get_object_at(self.world,
                                                    self.cursor_position[:2],
                                                    tolerance=tolerance)


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

import typing

import numpy as np
from PySide6.QtCore import QEvent, QPointF, QSize, Qt, Slot
from PySide6.QtGui import QHideEvent, QImage, QMouseEvent, QWheelEvent
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import QWidget

from .. import Image, Vector, VectorLike, World
from .camera import Camera
from .renderer import Renderer
from .utils import init, run, to_3d


class WorldView(QOpenGLWidget):

    def __init__(self,
                 parent: QWidget | None = None,
                 /,
                 world: World | None = None,
                 fps: typing.SupportsFloat = 30.0,
                 update_world: bool = False,
                 time_step: typing.SupportsFloat = 0.0,
                 factor: typing.SupportsFloat = 1.0,
                 helpers: bool = True,
                 camera_reset: bool = False,
                 camera_position: VectorLike = np.zeros(2),
                 camera_altitude: typing.SupportsFloat = 0.0,
                 camera_yaw: typing.SupportsFloat = 0.0,
                 camera_pitch: typing.SupportsFloat = 0.0,
                 camera_is_ortho: bool = False,
                 walls_height: typing.SupportsFloat = 10.0) -> None:
        QOpenGLWidget.__init__(self, parent)
        self._last_pos = QPointF()
        self._world = world
        self.camera = Camera()
        self.camera.is_ortho = camera_is_ortho
        if camera_reset:
            self.camera.reset(world)
        else:
            self.camera.position = to_3d(camera_position, camera_altitude)
            self.camera_yaw = camera_yaw
            self.camera_pitch = camera_pitch
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
        fps = float(fps)
        if fps > 0:
            self._timer_period = 1.0 / fps
            self.startTimer(int(self._timer_period * 1e3))
        if update_world:
            self.start_updating_world(time_step, factor)

    @property
    def camera_altitude(self) -> float:
        return float(self.camera.position[2])

    @camera_altitude.setter
    def camera_altitude(self, value: typing.SupportsFloat) -> None:
        self.camera.position[2] = float(value)

    @property
    def camera_is_ortho(self) -> bool:
        return self.camera.is_ortho

    @camera_is_ortho.setter
    def camera_is_ortho(self, value: bool) -> None:
        self.camera.is_ortho = True

    @property
    def camera_pitch(self) -> float:
        return self.camera.pitch

    @camera_pitch.setter
    def camera_pitch(self, value: typing.SupportsFloat) -> None:
        if not self.camera.is_ortho:
            self.camera.pitch = float(value)

    @property
    def camera_yaw(self) -> float:
        return self.camera.yaw

    @camera_yaw.setter
    def camera_yaw(self, value: typing.SupportsFloat) -> None:
        self.camera.user_yaw = self.camera.yaw = float(value)

    @property
    def camera_pose(self) -> tuple[Vector, float, float, float]:
        return (self.camera.position[:2], self.camera.position[2],
                self.camera.yaw, self.camera.pitch)

    @camera_pose.setter
    def camera_pose(
        self, value: tuple[VectorLike, typing.SupportsFloat,
                           typing.SupportsFloat, typing.SupportsFloat]
    ) -> None:
        self.camera.position = to_3d(*value[:2])
        self.camera_yaw = value[2]
        self.camera.pitch = float(value[3])

    @property
    def camera_position(self) -> Vector:
        return self.camera.position[:2]

    @camera_position.setter
    def camera_position(self, value: VectorLike) -> None:
        self.camera.position[:2] = np.asarray(value)

    def move_camera(self,
                    target_position: VectorLike,
                    target_altitude: typing.SupportsFloat = 0.0,
                    target_distance: typing.SupportsFloat = 30.0,
                    yaw: typing.SupportsFloat | None = None,
                    pitch: typing.SupportsFloat | None = None) -> None:
        if yaw is not None:
            self.camera_yaw = yaw
        if pitch is not None:
            self.camera_pitch = pitch
        self.camera.move(to_3d(target_position, target_altitude),
                         float(target_distance))

    def point_camera(self,
                     target_position: VectorLike,
                     target_altitude: typing.SupportsFloat = 0.0,
                     position: VectorLike | None = None,
                     altitude: typing.SupportsFloat | None = None) -> None:
        if self.camera.is_ortho:
            return
        if position is not None:
            self.camera.position[:2] = np.asarray(position)
        if altitude is not None:
            self.camera.position[2] = float(altitude)
        self.camera.point(to_3d(target_position, target_altitude))

    def reset_camera(self) -> None:
        self.camera.reset(self.world)

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
                               self.camera.matrix, self.camera.projection)

    def resizeGL(self, width: int, height: int) -> None:
        self.camera.set_viewport(width, height)

    def hideEvent(self, event: QHideEvent) -> None:
        self.cleanup()
        super().hideEvent(event)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        self._last_pos = event.position()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        pos = event.position()
        dx = pos.x() - self._last_pos.x()
        dy = pos.y() - self._last_pos.y()
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

        self._last_pos = pos

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


def run_in_viewer(self: World,
                  /,
                  fps: typing.SupportsFloat = 30,
                  time_step: typing.SupportsFloat = 0,
                  factor: typing.SupportsFloat = 1,
                  helpers: bool = True,
                  camera_reset: bool = False,
                  camera_position: VectorLike = (0, 0),
                  camera_altitude: typing.SupportsFloat = 0,
                  camera_yaw: typing.SupportsFloat = 0,
                  camera_pitch: typing.SupportsFloat = 0,
                  camera_is_ortho: bool = False,
                  walls_height: typing.SupportsFloat = 10,
                  duration: typing.SupportsFloat = -1) -> None:
    init()
    viewer = WorldView(world=self,
                       fps=fps,
                       update_world=True,
                       time_step=time_step,
                       factor=factor,
                       helpers=helpers,
                       camera_reset=camera_reset,
                       camera_position=camera_position,
                       camera_altitude=camera_altitude,
                       camera_yaw=camera_yaw,
                       camera_pitch=camera_pitch,
                       camera_is_ortho=camera_is_ortho,
                       walls_height=walls_height)
    viewer.setWindowTitle("PyEnki Viewer")
    viewer.show()
    run(float(duration) / float(factor))

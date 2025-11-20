from __future__ import annotations

from collections.abc import Callable
from typing import SupportsFloat, Unpack, cast, TYPE_CHECKING

import numpy as np
import numpy.typing

from .. import PhysicalObject, Vector, VectorLike, World
from .types import CameraConfig, Pixel, Vector3, Vector3Like

if TYPE_CHECKING:
    from PySide6.QtGui import QMatrix4x4


def to_3d(xy: VectorLike, z: SupportsFloat) -> Vector3:
    return np.concatenate([np.asarray(xy), [float(z)]])


def rotate(v: Vector3, delta: float) -> Vector3:
    if not delta:
        return v
    return np.array((np.cos(delta) * v[0] - np.sin(delta) * v[1],
                     -np.sin(delta) * v[0] + np.cos(delta) * v[1], v[2]),
                    dtype=v.dtype)


class Camera:

    def __init__(self) -> None:
        self.position: Vector3 = cast('Vector3', np.zeros(3))
        self._viewport: tuple[float, float] | None = None
        self.yaw = 0.0
        self.pitch = 0.0
        self.is_ortho = False
        self.fov = 1.0
        self.near_distance = 1.0
        self.far_distance = 1000.0

    def reset(self, world: World | None = None) -> None:
        self.yaw = np.pi / 2
        if self.is_ortho:
            self.pitch = -np.pi / 2
        else:
            self.pitch = -3 * np.pi / 8

        if world:
            self.position = np.array(
                (world.width * 0.5, max(0, -world.radius * 0.9),
                 max(world.radius * 2, world.width, world.height)))
        else:
            self.position = cast('Vector3', np.zeros(3))

    @property
    def matrix(self) -> QMatrix4x4:
        from PySide6.QtGui import QMatrix4x4

        m = QMatrix4x4()
        m.rotate(-self.pitch * 180 / np.pi - 90, 1, 0, 0)
        m.rotate(-self.yaw * 180 / np.pi + 90, 0, 0, 1)
        m.translate(*(-self.position))
        return m

    def set_viewport(self, width: float, height: float) -> None:
        self._viewport = (width, height)

    @property
    def projection(self) -> QMatrix4x4:
        from PySide6.QtGui import QMatrix4x4

        assert self._viewport is not None
        p = QMatrix4x4()
        aspect_ratio = self._viewport[0] / self._viewport[1]
        if self.is_ortho:
            s = 0.5 * np.tan(self.fov) * self.position[2]
            p.ortho(-s * aspect_ratio, s * aspect_ratio, -s, s,
                    self.near_distance, self.far_distance)
        else:
            p.perspective(self.fov * 180 / np.pi, aspect_ratio,
                          self.near_distance, self.far_distance)
        return p

    @property
    def forward(self) -> Vector3:
        if self.is_ortho:
            return np.array((0, 0, -1))
        else:
            return np.array(
                (np.cos(self.yaw) * np.cos(self.pitch),
                 np.sin(self.yaw) * np.cos(self.pitch), np.sin(self.pitch)))

    @property
    def left(self) -> Vector3:
        return np.array((-np.sin(self.yaw), np.cos(self.yaw), 0.0))

    @property
    def up(self) -> Vector3:
        return cast('Vector3', np.cross(self.forward, self.left))

    def move(self,
             target_position: Vector3Like,
             target_distance: float = 30.0) -> None:
        self.position = np.asarray(
            target_position) - target_distance * self.forward

    def point(self, target_position: Vector3Like) -> None:
        if self.is_ortho:
            return
        delta = np.asarray(target_position) - self.position
        self.yaw = np.atan2(delta[1], delta[0])
        self.pitch = np.atan2(delta[2], np.linalg.norm(delta[:2]))

    def depth_to_z(self, depth: float) -> float:
        return self.far_distance * self.near_distance / (
            self.far_distance - depth *
            (self.far_distance - self.near_distance))


CameraCallback = Callable[[Camera, World], None]


class HasCamera:

    world: World | None = None

    def __init__(self,
                 /,
                 world: World | None = None,
                 **config: Unpack[CameraConfig]) -> None:
        self.camera = Camera()
        self.update_camera_config(**config)
        self._tracked_object: PhysicalObject | None = None
        self._camera_callback: CameraCallback | None = None
        self.tracking_distance = 0.0
        self.tracking_angle = 0.0

    @property
    def camera_matrix(self) -> QMatrix4x4:
        return self.camera.matrix

    @property
    def camera_projection(self) -> QMatrix4x4:
        return self.camera.projection

    @property
    def camera_config(self) -> CameraConfig:
        return {
            'camera_position': self.camera.position[:2],
            'camera_altitude': self.camera.position[2],
            'camera_yaw': self.camera.yaw,
            'camera_pitch': self.camera.pitch,
            'camera_is_ortho': self.camera.is_ortho
        }

    def update_camera_config(self, **config: Unpack[CameraConfig]) -> None:
        if 'camera_position' in config:
            self.camera.position[:2] = np.asarray(config['camera_position'])
        if 'camera_altitude' in config:
            self.camera.position[2] = float(config['camera_altitude'])
        if 'camera_yaw' in config:
            self.camera.yaw = float(config['camera_yaw'])
        if 'camera_pitch' in config:
            self.camera.pitch = float(config['camera_pitch'])
        if 'camera_is_ortho' in config:
            self.camera.is_ortho = config['camera_is_ortho']
        if config.get('camera_reset', False):
            self.reset_camera()

    @property
    def camera_altitude(self) -> float:
        return float(self.camera.position[2])

    @camera_altitude.setter
    def camera_altitude(self, value: SupportsFloat) -> None:
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
    def camera_pitch(self, value: SupportsFloat) -> None:
        if not self.camera.is_ortho:
            self.camera.pitch = float(value)

    @property
    def camera_yaw(self) -> float:
        return self.camera.yaw

    @camera_yaw.setter
    def camera_yaw(self, value: SupportsFloat) -> None:
        self.camera.yaw = float(value)

    @property
    def camera_position(self) -> Vector:
        return self.camera.position[:2]

    @camera_position.setter
    def camera_position(self, value: VectorLike) -> None:
        self.camera.position[:2] = np.asarray(value)

    # @property
    # def camera_pose(self) -> tuple[Vector, float, float, float]:
    #     return (self.camera.position[:2], self.camera.position[2],
    #             self.camera.yaw, self.camera.pitch)

    # @camera_pose.setter
    # def camera_pose(
    #     self, value: tuple[VectorLike, SupportsFloat, SupportsFloat,
    #                        SupportsFloat]
    # ) -> None:
    #     self.camera.position = to_3d(*value[:2])
    #     self.camera_yaw = value[2]
    #     self.camera.pitch = float(value[3])

    def move_camera(self,
                    target_position: VectorLike,
                    target_altitude: SupportsFloat = 0.0,
                    target_distance: SupportsFloat = 30.0,
                    yaw: SupportsFloat | None = None,
                    pitch: SupportsFloat | None = None) -> None:
        """
        Move the camera so to point towards the target.
        Same interface as :py:meth:`pyenki.WorldView.move_camera`.

        :param      target_position:  The target horizontal position in cm.
        :param      target_altitude:  The target vertical position in cm.
        :param      target_distance:  The distance to the target.
        :param      yaw:              Optionally sets the camera yaw.
        :param      pitch:            Optionally sets the camera pitch.
        """
        if yaw is not None:
            self.camera_yaw = yaw
        if pitch is not None:
            self.camera_pitch = pitch
        self.camera.move(to_3d(target_position, target_altitude),
                         float(target_distance))

    def point_camera(self,
                     target_position: VectorLike,
                     target_altitude: SupportsFloat = 0.0,
                     position: VectorLike | None = None,
                     altitude: SupportsFloat | None = None) -> None:
        """
        Rotate the camera so to point towards the target.
        Same interface as :py:meth:`pyenki.WorldView.point_camera`.

        :param      target_position:  The target horizontal position in cm.
        :param      target_altitude:  The target vertical position in cm.
        :param      position:       Optionally sets the camera position.
        :param      altitude:     Optionally sets the camera altitude.
        """
        if self.camera.is_ortho:
            return
        if position is not None:
            self.camera.position[:2] = np.asarray(position)
        if altitude is not None:
            self.camera.position[2] = float(altitude)
        self.camera.point(to_3d(target_position, target_altitude))

    def reset_camera(self) -> None:
        if self.world:
            self.camera.reset(self.world)

    def get_position_of_pixel(self, pixel: Pixel) -> Vector3 | None:
        return None

    def update_tracking(self) -> None:
        if self.tracked_object:
            self.camera.yaw = self.tracked_object.angle + self.tracking_angle
            self.camera.move(
                to_3d(self.tracked_object.position,
                      self.tracked_object.height), self.tracking_distance)

    @property
    def tracked_object(self) -> PhysicalObject | None:
        return self._tracked_object

    @tracked_object.setter
    def tracked_object(self, value: PhysicalObject | None) -> None:
        if self._tracked_object is value:
            return
        if not self._tracked_object and value:
            self._previous_camera_config = self.camera_config
            self.tracking_distance = value.radius * 5
            self.tracking_angle = 0
            self._tracked_object = value
            self.update_tracking()
        else:
            if not value:
                self.update_camera_config(**self._previous_camera_config)
            self._tracked_object = value

    def update_camera(self) -> None:
        self.update_tracking()
        if self._camera_callback and self.world:
            self._camera_callback(self.camera, self.world)

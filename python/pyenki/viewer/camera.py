from __future__ import annotations

import sys
from collections.abc import Callable
from typing import TYPE_CHECKING, SupportsFloat, cast

if sys.version_info >= (3, 11):
    from typing import Unpack
else:
    from typing_extensions import Unpack

import numpy as np
import numpy.typing

from .. import PhysicalObject, Vector, VectorLike, World
from .types import CameraConfig, Pixel, Vector3, Vector3Like

if TYPE_CHECKING:
    from PySide6.QtGui import QMatrix4x4


def to_3d(xy: VectorLike, z: SupportsFloat) -> Vector3:
    """
    Concatenate a 2D vector and an vertical component as a 3D vector

    :param      xy:   The horizontal vector ``(x, y)``
    :param      z:    The vertical component

    :returns:   The concatenation ``(x, y, z)``
    """
    return np.concatenate([np.asarray(xy), [float(z)]])


def rotate(value: Vector3, angle: float) -> Vector3:
    """
    Rotates a 3D vector around the vertical axis

    :param      value:      The vector
    :param      angle:      The rotation angle in rad.

    :returns:   The rotated vector
    """
    if not angle:
        return value
    return np.array(
        (np.cos(angle) * value[0] - np.sin(angle) * value[1],
         -np.sin(angle) * value[0] + np.cos(angle) * value[1], value[2]),
        dtype=value.dtype)


class Camera:
    """
    This class describes a camera.

    Attributes:
        position (Vector3): the position of the camera in cm.
        yaw (float): the rotation around the vertical axis in rad.
        pitch (float): the rotation around the lateral axis in rad.
        is_ortho (bool): whether it uses orthographic projection.
        fov (float): the vertical field of view in rad.
        near_distance (float): the near clipping distance in cm.
        far_distance (float): the far clipping distance in cm.
        view_port (tuple[float, float] | None): the size of the viewport in pixels if set.
        forward (Vector3): The unit vector in the forward direction (readonly).
        up (Vector3): The unit vector in the up direction (readonly).
        left (Vector3): The unit vector in the left direction (readonly).
    """

    def __init__(self,
                 position: Vector3 = cast('Vector3', np.zeros(3)),
                 yaw: float = 0,
                 pitch: float = 0,
                 is_ortho: bool = False) -> None:
        """
        Constructs a new instance.

        :param      position:  The position
        :param      yaw:       The yaw
        :param      pitch:     The pitch
        :param      is_ortho:  Whether to use orthographic projection
        """
        self.position = position
        self.viewport: tuple[float, float] | None = None
        self.yaw = yaw
        self.pitch = pitch
        self.is_ortho = is_ortho
        self.fov = 1.0
        self.near_distance = 1.0
        self.far_distance = 1000.0

    def reset(self, world: World | None = None) -> None:
        """
        Resets the camera to capture the world from a default POV.

        :param      world:  An optional The world
        """
        self.yaw = np.pi / 2
        if self.is_ortho:
            self.pitch = -np.pi / 2
        else:
            self.pitch = -3 * np.pi / 8

        if world:
            self.position = np.array(
                (world.lx * 0.5, max(0, -world.radius * 0.9),
                 max(world.radius * 2, world.lx, world.ly)))
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
        """
        Sets the viewport.

        :param      width:   The width in pixels
        :param      height:  The height in pixels
        """
        self.viewport = (width, height)

    @property
    def projection(self) -> QMatrix4x4:
        from PySide6.QtGui import QMatrix4x4

        assert self.viewport is not None
        p = QMatrix4x4()
        aspect_ratio = self.viewport[0] / self.viewport[1]
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
        """
        Translates the camera to point a target
        while maintaining its attitude.

        :param      target_position:  The target position
        :param      target_distance:  The target distance
        """
        self.position = np.asarray(
            target_position) - target_distance * self.forward

    def point(self, target_position: Vector3Like) -> None:
        """
        Rotates the camera to point a target while
        maintaining its position.

        :param      target_position:  The target position
        """
        if self.is_ortho:
            return
        delta = np.asarray(target_position) - self.position
        self.yaw = np.atan2(delta[1], delta[0])
        self.pitch = np.atan2(delta[2], np.linalg.norm(delta[:2]))

    def depth_to_z(self, depth: float) -> float:
        return self.far_distance * self.near_distance / (
            self.far_distance - depth *
            (self.far_distance - self.near_distance))

    @property
    def config(self) -> CameraConfig:
        """
        Returns the relevant camera attributes as a dictionary

        :returns:   The camera configuration.
        """
        return {
            'camera_position': self.position[:2],
            'camera_altitude': self.position[2],
            'camera_yaw': self.yaw,
            'camera_pitch': self.pitch,
            'camera_is_ortho': self.is_ortho
        }


CameraCallback = Callable[[Camera, World], None]


class HasCamera:
    """
    Mixins for classes that hold and manipulate a camera, like
    :py:class:`pyenki.viewer.WorldView` and
    :py:class:`pyenki.buffer.EnkiRemoteFrameBuffer`
    """

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
        """
        The transformation matrix between camera frame to world frame
        """
        return self.camera.matrix

    @property
    def camera_projection(self) -> QMatrix4x4:
        """
        The projection matrix
        """
        return self.camera.projection

    @property
    def camera_config(self) -> CameraConfig:
        """
        The camera configuration
        """
        return self.camera.config

    def update_camera_config(self, **config: Unpack[CameraConfig]) -> None:
        """
        Updates the camera configuration

        :param config:  The (potentially partial) configuration
        """
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
        """
        The camera vertical position (in cm).
        """
        return float(self.camera.position[2])

    @camera_altitude.setter
    def camera_altitude(self, value: SupportsFloat) -> None:
        self.camera.position[2] = float(value)

    @property
    def camera_is_ortho(self) -> bool:
        """
        Whether the camera uses an orthographic projection
        """
        return self.camera.is_ortho

    @camera_is_ortho.setter
    def camera_is_ortho(self, value: bool) -> None:
        self.camera.is_ortho = True

    @property
    def camera_pitch(self) -> float:
        """
        The camera pitch (in rad).
        """
        return self.camera.pitch

    @camera_pitch.setter
    def camera_pitch(self, value: SupportsFloat) -> None:
        if not self.camera.is_ortho:
            self.camera.pitch = float(value)

    @property
    def camera_yaw(self) -> float:
        """
        The camera yaw (in rad).
        """
        return self.camera.yaw

    @camera_yaw.setter
    def camera_yaw(self, value: SupportsFloat) -> None:
        self.camera.yaw = float(value)

    @property
    def camera_position(self) -> Vector:
        """
        The camera horizontal position (in cm).
        """
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
        Translates the camera to point a target
        while maintaining its attitude.

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
        Rotates the camera to point a target while maintaining its position.

        :param      target_position:  The target horizontal position in cm.
        :param      target_altitude:  The target vertical position in cm.
        :param      position:         Optionally sets the camera position.
        :param      altitude:         Optionally sets the camera altitude.
        """
        if self.camera.is_ortho:
            return
        if position is not None:
            self.camera.position[:2] = np.asarray(position)
        if altitude is not None:
            self.camera.position[2] = float(altitude)
        self.camera.point(to_3d(target_position, target_altitude))

    def reset_camera(self) -> None:
        """
        Resets the camera configuration
        """
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
    def is_tracking(self) -> bool:
        """
        Whether it is tracking an object
        """
        return self.tracked_object is not None

    @property
    def tracked_object(self) -> PhysicalObject | None:
        """
        The object being tracked
        """
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

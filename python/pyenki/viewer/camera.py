from __future__ import annotations

from typing import Annotated, TypeAlias, cast

import numpy as np
import numpy.typing
from PySide6.QtGui import QMatrix4x4

from .. import World

Vector3: TypeAlias = Annotated[numpy.typing.NDArray[numpy.float64], '[3, 1]']
Vector3Like: TypeAlias = Annotated[numpy.typing.ArrayLike, numpy.float64,
                                   '[3, 1]']


class Camera:

    def __init__(self) -> None:
        self.position: Vector3 = cast(Vector3, np.zeros(3))
        self._matrix = QMatrix4x4()
        self._viewport: tuple[float, float] | None = None
        self._proj = QMatrix4x4()
        self.yaw = 0.0
        self.user_yaw = 0.0
        self.pitch = 0.0
        self.is_ortho = False
        self.fov = 1.0
        # self.update_projection()
        # self.update_matrix()

    def reset(self, world: World | None = None) -> None:
        self.yaw = self.user_yaw = np.pi / 2
        if self.is_ortho:
            self.pitch = -np.pi / 2
        else:
            self.pitch = -3 * np.pi / 8

        if world:
            self.position = np.array(
                (world.width * 0.5, max(0, -world.radius * 0.9),
                 max(world.radius * 2, world.width, world.height)))
        else:
            self.position = cast(Vector3, np.zeros(3))

    @property
    def matrix(self) -> QMatrix4x4:
        self.update_matrix()
        return self._matrix

    def update_matrix(self) -> None:
        self._matrix.setToIdentity()
        self._matrix.rotate(-self.pitch * 180 / np.pi - 90, 1, 0, 0)
        self._matrix.rotate(-self.yaw * 180 / np.pi + 90, 0, 0, 1)
        self._matrix.translate(*(-self.position))

    def update_projection(self) -> None:
        if not self._viewport:
            return
        self._proj.setToIdentity()
        aspect_ratio = self._viewport[0] / self._viewport[1]
        if self.is_ortho:
            s = 0.5 * np.tan(self.fov) * self.position[2]
            # print(-s * aspect_ratio, s * aspect_ratio, -s, s, 1.0, 1000.0)
            self._proj.ortho(-s * aspect_ratio, s * aspect_ratio, -s, s, 1.0,
                             1000.0)
        else:
            self._proj.perspective(self.fov * 180 / np.pi, aspect_ratio, 1.0,
                                   1000.0)

    def set_viewport(self, width: float, height: float) -> None:
        self._viewport = (width, height)
        # self.update_projection()

    @property
    def projection(self) -> QMatrix4x4:
        self.update_projection()
        return self._proj

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
        return cast(Vector3, np.cross(self.forward, self.left))

    def move(self,
             target_position: Vector3Like,
             target_distance: float = 30.0) -> None:
        self.position = np.asarray(
            target_position) - target_distance * self.forward
        self.update_matrix()

    def point(self, target_position: Vector3Like) -> None:
        if self.is_ortho:
            return
        delta = np.asarray(target_position) - self.position
        self.yaw = np.atan2(delta[1], delta[0])
        self.pitch = np.atan2(delta[2], np.linalg.norm(delta[:2]))
        self.update_matrix()

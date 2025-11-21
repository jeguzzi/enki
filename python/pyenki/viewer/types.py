from __future__ import annotations

import sys
from collections.abc import Callable
from typing import (Annotated, Any, Protocol, SupportsFloat, SupportsInt,
                    TypeAlias, TypedDict)

if sys.version_info >= (3, 11):
    from typing import NotRequired, Unpack
else:
    from typing_extensions import Unpack, NotRequired

import numpy
import numpy.typing

from .. import Image, PhysicalObject, Vector, VectorLike, World

Vector3: TypeAlias = Annotated[numpy.typing.NDArray[numpy.float64], '[3, 1]']
Vector3Like: TypeAlias = Annotated[numpy.typing.ArrayLike, numpy.float64,
                                   '[3, 1]']
Pixel: TypeAlias = tuple[float, float]
PositionOfPixelGetterProtocol: TypeAlias = Callable[[Pixel], Vector3 | None]


class CameraConfig(TypedDict):
    camera_position: NotRequired[VectorLike]
    camera_altitude: NotRequired[SupportsFloat]
    camera_yaw: NotRequired[SupportsFloat]
    camera_pitch: NotRequired[SupportsFloat]
    camera_is_ortho: NotRequired[bool]
    camera_reset: NotRequired[bool]


class HasCameraProtocol(Protocol):

    world: World | None = None

    def __init__(self,
                 /,
                 world: World | None = None,
                 **config: Unpack[CameraConfig]) -> None:
        ...

    @property
    def camera_config(self) -> CameraConfig:
        ...

    def update_camera_config(self, **config: Unpack[CameraConfig]) -> None:
        ...

    @property
    def camera_altitude(self) -> float:
        ...

    @camera_altitude.setter
    def camera_altitude(self, value: SupportsFloat) -> None:
        ...

    @property
    def camera_is_ortho(self) -> bool:
        ...

    @camera_is_ortho.setter
    def camera_is_ortho(self, value: bool) -> None:
        ...

    @property
    def camera_pitch(self) -> float:
        ...

    @camera_pitch.setter
    def camera_pitch(self, value: SupportsFloat) -> None:
        ...

    @property
    def camera_yaw(self) -> float:
        ...

    @camera_yaw.setter
    def camera_yaw(self, value: SupportsFloat) -> None:
        ...

    @property
    def camera_position(self) -> Vector:
        ...

    @camera_position.setter
    def camera_position(self, value: VectorLike) -> None:
        ...

    def move_camera(self,
                    target_position: VectorLike,
                    target_altitude: SupportsFloat = 0.0,
                    target_distance: SupportsFloat = 30.0,
                    yaw: SupportsFloat | None = None,
                    pitch: SupportsFloat | None = None) -> None:
        ...

    def point_camera(self,
                     target_position: VectorLike,
                     target_altitude: SupportsFloat = 0.0,
                     position: VectorLike | None = None,
                     altitude: SupportsFloat | None = None) -> None:
        ...

    def reset_camera(self) -> None:
        ...


class WorldViewProtocol(HasCameraProtocol, Protocol):

    walls_height: float
    world: World | None

    def __init__(self,
                 parent: Any = None,
                 /,
                 world: World | None = None,
                 fps: SupportsFloat = 30.0,
                 update_world: bool = False,
                 time_step: SupportsFloat = 0.0,
                 factor: SupportsFloat = 1.0,
                 helpers: bool = True,
                 walls_height: SupportsFloat = 10.0,
                 **camera_config: Unpack[CameraConfig]) -> None:
        ...

    def save_image(self, path: str) -> None:
        ...

    @property
    def image(self) -> Image:
        ...

    def start_updating_world(self,
                             time_step: SupportsFloat = 0.0,
                             factor: SupportsFloat = 1.0) -> None:
        ...

    def show(self) -> None:
        ...

    def hide(self) -> None:
        ...

    @property
    def pyside_widget(self) -> object:
        ...

    @property
    def pyqt_widget(self) -> object:
        ...


class InitProtocol(Protocol):

    def __call__(self, share: bool = True) -> None:
        ...


class RunProtocol(Protocol):

    def __call__(self, duration: SupportsFloat = -1) -> None:
        ...


class RenderProtocol(Protocol):

    def __call__(self,
                 world: World,
                 /,
                 walls_height: SupportsFloat = 10,
                 width: SupportsInt = 640,
                 height: SupportsInt = 360,
                 selected_object: PhysicalObject | None = None,
                 **camera_config: Unpack[CameraConfig]) -> Image:
        ...


class SaveImageProtocol(Protocol):

    def __call__(self,
                 world: World,
                 path: str,
                 /,
                 walls_height: SupportsFloat = 10,
                 width: SupportsInt = 640,
                 height: SupportsInt = 360,
                 selected_object: PhysicalObject | None = None,
                 **camera_config: Unpack[CameraConfig]) -> None:
        ...


class RunInViewerProtocol(Protocol):

    def __call__(self,
                 world: World,
                 /,
                 fps: SupportsFloat = 30,
                 time_step: SupportsFloat = 0,
                 factor: SupportsFloat = 1,
                 helpers: bool = True,
                 walls_height: SupportsFloat = 10,
                 duration: SupportsFloat = -1,
                 **camera_config: Unpack[CameraConfig]) -> None:
        ...

from __future__ import annotations

import importlib.util
import os
import typing
import warnings
from collections.abc import Callable

import numpy as np

from .. import Image, Vector, VectorLike, World


class WorldViewProtocol(typing.Protocol):

    def __init__(self,
                 parent: typing.Any = None,
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
        ...

    @property
    def camera_altitude(self) -> float:
        ...

    @camera_altitude.setter
    def camera_altitude(self, value: typing.SupportsFloat) -> None:
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
    def camera_pitch(self, value: typing.SupportsFloat) -> None:
        ...

    @property
    def camera_yaw(self) -> float:
        ...

    @camera_yaw.setter
    def camera_yaw(self, value: typing.SupportsFloat) -> None:
        ...

    @property
    def camera_pose(self) -> tuple[Vector, float, float, float]:
        ...

    @camera_pose.setter
    def camera_pose(
        self, value: tuple[VectorLike, typing.SupportsFloat,
                           typing.SupportsFloat, typing.SupportsFloat]
    ) -> None:
        ...

    @property
    def camera_position(self) -> Vector:
        ...

    @camera_position.setter
    def camera_position(self, value: VectorLike) -> None:
        ...

    def move_camera(self,
                    target_position: VectorLike,
                    target_altitude: typing.SupportsFloat = 0.0,
                    target_distance: typing.SupportsFloat = 30.0,
                    yaw: typing.SupportsFloat | None = None,
                    pitch: typing.SupportsFloat | None = None) -> None:
        ...

    def point_camera(self,
                     target_position: VectorLike,
                     target_altitude: typing.SupportsFloat = 0.0,
                     position: VectorLike | None = None,
                     altitude: typing.SupportsFloat | None = None) -> None:
        ...

    def reset_camera(self) -> None:
        ...

    def save_image(self, path: str) -> None:
        ...

    @property
    def image(self) -> Image:
        ...

    @property
    def walls_height(self) -> float:
        ...

    @walls_height.setter
    def walls_height(self, value: float) -> None:
        ...

    @property
    def widget(self) -> typing.Self:
        ...

    @property
    def world(self) -> World | None:
        ...

    @world.setter
    def world(self, value: World | None) -> None:
        ...

    def start_updating_world(self,
                             time_step: typing.SupportsFloat = 0.0,
                             factor: typing.SupportsFloat = 1.0) -> None:
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


class InitType(typing.Protocol):

    def __call__(self, share: bool = True) -> None:
        ...


class RunType(typing.Protocol):

    def __call__(self, duration: typing.SupportsFloat = -1) -> None:
        ...


class RenderType(typing.Protocol):

    def __call__(self,
                 /,
                 world: World,
                 camera_reset: bool = False,
                 camera_position: VectorLike = (0, 0),
                 camera_altitude: typing.SupportsFloat = 0,
                 camera_yaw: typing.SupportsFloat = 0,
                 camera_pitch: typing.SupportsFloat = 0,
                 camera_is_ortho: bool = False,
                 walls_height: typing.SupportsFloat = 10,
                 width: typing.SupportsInt = 640,
                 height: typing.SupportsInt = 360) -> Image:
        ...


class SaveImageType(typing.Protocol):

    def __call__(self,
                 world: World,
                 path: str,
                 /,
                 camera_reset: bool = False,
                 camera_position: VectorLike = (0, 0),
                 camera_altitude: typing.SupportsFloat = 0,
                 camera_yaw: typing.SupportsFloat = 0,
                 camera_pitch: typing.SupportsFloat = 0,
                 camera_is_ortho: bool = False,
                 walls_height: typing.SupportsFloat = 10,
                 width: typing.SupportsInt = 640,
                 height: typing.SupportsInt = 360) -> None:
        ...


class RunInViewerType(typing.Protocol):

    def __call__(self,
                 world: World,
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
        ...


_use_native_viewer: bool | None = None
WorldView: type[WorldViewProtocol]
init: InitType
run: RunType
render: RenderType
save_image: SaveImageType
run_in_viewer: RunInViewerType
cleanup: Callable[[], None]


def has_native_viewer() -> bool:
    return importlib.util.find_spec("pyenki.pyenki_viewer") is not None


def _get_env_native_viewer() -> bool:
    value = 'PYENKI_NATIVE_VIEWER' in os.environ
    if not has_native_viewer() and value:
        warnings.warn("Pyenki without native viewer")
        return False
    return value


def patch_world() -> None:
    World.save_image = save_image  # type: ignore
    World.render = render  # type: ignore
    World.run_in_viewer = run_in_viewer  # type: ignore


def use_native_viewer(value: bool) -> None:
    global _use_native_viewer
    if value is not _use_native_viewer:
        if value and not has_native_viewer():
            warnings.warn("Pyenki without native viewer")
            return
        global WorldView
        global init
        global run
        global cleanup
        global save_image
        global run_in_viewer
        global render
        if value:
            print('Use native viewer')

            from . import native

            def save_image(world: World,
                           path: str,
                           /,
                           camera_reset: bool = False,
                           camera_position: VectorLike = (0, 0),
                           camera_altitude: typing.SupportsFloat = 0,
                           camera_yaw: typing.SupportsFloat = 0,
                           camera_pitch: typing.SupportsFloat = 0,
                           camera_is_ortho: bool = False,
                           walls_height: typing.SupportsFloat = 10,
                           width: typing.SupportsInt = 640,
                           height: typing.SupportsInt = 360) -> None:
                native.save_image(world, path, camera_reset, camera_position,
                                  camera_altitude, camera_yaw, camera_pitch,
                                  camera_is_ortho, walls_height, width, height)

            def render(world: World,
                       /,
                       camera_reset: bool = False,
                       camera_position: VectorLike = (0, 0),
                       camera_altitude: typing.SupportsFloat = 0,
                       camera_yaw: typing.SupportsFloat = 0,
                       camera_pitch: typing.SupportsFloat = 0,
                       camera_is_ortho: bool = False,
                       walls_height: typing.SupportsFloat = 10,
                       width: typing.SupportsInt = 640,
                       height: typing.SupportsInt = 360) -> Image:
                return native.render(world, camera_reset, camera_position,
                                     camera_altitude, camera_yaw, camera_pitch,
                                     camera_is_ortho, walls_height, width,
                                     height)

            def run_in_viewer(world: World,
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
                native.run_in_viewer(world, fps, time_step, factor, helpers,
                                     camera_reset, camera_position,
                                     camera_altitude, camera_yaw, camera_pitch,
                                     camera_is_ortho, walls_height, duration)

            WorldView = native.WorldView  # type: ignore[assignment]
            init = native.init
            run = native.run
            cleanup = native.cleanup
        else:
            print('Use python viewer')

            from . import offscreen_renderer, utils, widget

            WorldView = widget.WorldView
            init = utils.init
            run = utils.run
            cleanup = utils.cleanup
            save_image = offscreen_renderer.save_image
            run_in_viewer = widget.run_in_viewer
            render = offscreen_renderer.render

        patch_world()


def is_using_native_viewer() -> bool:
    assert _use_native_viewer is not None
    return _use_native_viewer


use_native_viewer(_get_env_native_viewer())

__all__ = ['init', 'run', 'cleanup', 'WorldView']

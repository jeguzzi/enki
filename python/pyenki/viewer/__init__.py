from __future__ import annotations

import importlib.util
import os
import sys
import warnings
from collections.abc import Callable
from typing import SupportsFloat, SupportsInt

if sys.version_info >= (3, 11):
    from typing import Unpack
else:
    from typing_extensions import Unpack

from .. import Image, PhysicalObject, World
from .types import (CameraConfig, InitProtocol, PositionOfPixelGetterProtocol,
                    RenderProtocol, RunInViewerProtocol, RunProtocol,
                    SaveImageProtocol, WorldViewProtocol)

_use_native_viewer: bool | None = None
WorldView: type[WorldViewProtocol]
init: InitProtocol
run: RunProtocol
render: RenderProtocol
save_image: SaveImageProtocol
run_in_viewer: RunInViewerProtocol
cleanup: Callable[[], None]
get_position_of_pixel: PositionOfPixelGetterProtocol


def has_native_viewer() -> bool:
    return importlib.util.find_spec("pyenki.pyenki_viewer") is not None


def has_pyside() -> bool:
    return importlib.util.find_spec("PySide6") is not None


def _get_env_native_viewer() -> bool:
    value = 'PYENKI_NATIVE_VIEWER' in os.environ
    p = has_pyside()
    n = has_native_viewer()
    if not p and not n:
        raise RuntimeError(
            "No PySide6 installed and Pyenki built without native viewer")
    if not p and not value:
        warnings.warn("No PySide6 installed, switch to native viewer",
                      stacklevel=2)
        return True
    if not n and value:
        warnings.warn("Pyenki without native viewer, switch to Python viewer",
                      stacklevel=2)
        return False
    return value


def patch_world() -> None:
    World.save_image = save_image  # type: ignore[attr-defined]
    World.render = render  # type: ignore[attr-defined]
    World.run_in_viewer = run_in_viewer  # type: ignore[attr-defined]


def use_native_viewer(value: bool) -> None:
    global _use_native_viewer
    if value is not _use_native_viewer:
        if value and not has_native_viewer():
            warnings.warn("Pyenki without native viewer", stacklevel=2)
            return
        global WorldView
        global init
        global run
        global cleanup
        global save_image
        global run_in_viewer
        global render
        global get_position_of_pixel
        if value:
            # print('Using native viewer')

            from . import native

            def save_image(world: World,
                           path: str,
                           /,
                           walls_height: SupportsFloat = 10,
                           width: SupportsInt = 640,
                           height: SupportsInt = 360,
                           selected_object: PhysicalObject | None = None,
                           **config: Unpack[CameraConfig]) -> None:
                native.save_image(world,
                                  path,
                                  walls_height=walls_height,
                                  width=width,
                                  height=height,
                                  selected_object=selected_object,
                                  **config)

            def render(world: World,
                       /,
                       walls_height: SupportsFloat = 10,
                       width: SupportsInt = 640,
                       height: SupportsInt = 360,
                       selected_object: PhysicalObject | None = None,
                       **config: Unpack[CameraConfig]) -> Image:
                return native.render(world,
                                     walls_height=walls_height,
                                     width=width,
                                     height=height,
                                     selected_object=selected_object,
                                     **config)

            def run_in_viewer(world: World,
                              /,
                              fps: SupportsFloat = 30,
                              time_step: SupportsFloat = 0,
                              factor: SupportsFloat = 1,
                              helpers: bool = True,
                              walls_height: SupportsFloat = 10,
                              duration: SupportsFloat = -1,
                              **config: Unpack[CameraConfig]) -> None:
                native.run_in_viewer(world,
                                     fps=fps,
                                     time_step=time_step,
                                     factor=factor,
                                     helpers=helpers,
                                     walls_height=walls_height,
                                     duration=duration,
                                     **config)

            WorldView = native.WorldView  # type: ignore[assignment]
            init = native.init
            run = native.run
            cleanup = native.cleanup
            get_position_of_pixel = native.get_position_of_pixel
        else:
            # print('Use python viewer')

            from . import offscreen_renderer, utils, widget

            WorldView = widget.WorldView
            init = utils.init
            run = utils.run
            cleanup = utils.cleanup
            save_image = offscreen_renderer.save_image
            run_in_viewer = widget.run_in_viewer
            render = offscreen_renderer.render
            get_position_of_pixel = offscreen_renderer.get_position_of_pixel

        patch_world()


def is_using_native_viewer() -> bool:
    assert _use_native_viewer is not None
    return _use_native_viewer


use_native_viewer(_get_env_native_viewer())

__all__ = ['init', 'run', 'cleanup', 'WorldView']

from __future__ import annotations

import threading
import typing
import weakref
from collections.abc import Iterator
from contextlib import contextmanager

from PySide6.QtGui import (QImage, QOffscreenSurface, QOpenGLContext,
                           QSurfaceFormat)
from PySide6.QtOpenGL import (QOpenGLFramebufferObject,
                              QOpenGLFramebufferObjectFormat)

from .. import Image, PhysicalObject, World
from .camera import HasCamera
from .renderer import Renderer
from .types import CameraConfig, Pixel, Vector3
from .utils import get_position_of_pixel as _get_position_of_pixel
from .utils import init, to_numpy_image


class OffScreenRenderer(HasCamera):

    def __init__(self,
                 share: bool = False,
                 **camera_config: typing.Unpack[CameraConfig]) -> None:
        HasCamera.__init__(self, **camera_config)
        # Add a check that we initialized
        if threading.current_thread() is not threading.main_thread():
            share = False
        self._thread_id = threading.current_thread().native_id
        self.context: QOpenGLContext | None = QOpenGLContext()
        if share:
            self.context.setShareContext(QOpenGLContext.globalShareContext())
        fmt = QSurfaceFormat.defaultFormat()
        # fmt.setDepthBufferSize(32)
        self.context.setFormat(fmt)
        self.context.create()
        if not self.context.isValid():
            raise RuntimeError("Unable to create context")
        self.surface = QOffscreenSurface()
        self.surface.setFormat(fmt)
        self.surface.create()
        if not self.surface.isValid():
            raise RuntimeError("Unable to create offscreen surface")
        self.fbo_format = QOpenGLFramebufferObjectFormat()
        self.fbo_format.setAttachment(
            QOpenGLFramebufferObject.Attachment.CombinedDepthStencil)
        self.fbo: QOpenGLFramebufferObject | None = None
        # width, height
        self.size: tuple[int, int] | None = None
        self.renderer = Renderer.get(self.context)
        self.context.destroyed.connect(self.context_destroyed)
        # self.context.aboutToBeDestroyed.connect(self.context_will_destroy)

    def context_destroyed(self) -> None:
        self.context = None

    def __del__(self) -> None:
        # print('OffScreenRenderer.__del__')
        if self._thread_id == threading.current_thread(
        ).native_id and self.context:
            self.context.makeCurrent(self.surface)
            self.renderer.context_will_be_destroyed(self.context)
            self.context.doneCurrent()

    def _update_size(self, width: int, height: int) -> None:
        size = (width, height)
        if size != self.size:
            self.size = size
            self.fbo = QOpenGLFramebufferObject(width, height, self.fbo_format)
            self.camera.set_viewport(width, height)

    @contextmanager
    def bind(self) -> Iterator[None]:
        assert self.context
        self.context.makeCurrent(self.surface)
        assert self.fbo
        self.fbo.bind()
        try:
            yield
        finally:
            self.fbo.release()
            self.context.doneCurrent()

    def draw(self,
             world: World,
             walls_height: typing.SupportsFloat = 10,
             width: typing.SupportsInt = 640,
             height: typing.SupportsInt = 360,
             selected_object: PhysicalObject | None = None,
             **camera_config: typing.Unpack[CameraConfig]) -> QImage:
        assert self.context
        assert self._thread_id == threading.current_thread().native_id
        width = int(width)
        height = int(height)
        self.context.makeCurrent(self.surface)
        self._update_size(width, height)
        assert self.fbo
        self.fbo.bind()
        self.context.functions().glViewport(0, 0, width, height)
        self.world = world
        self.update_camera_config(**camera_config)
        self.world = None
        self.renderer.draw(world, float(walls_height), self.camera.matrix,
                           self.camera.projection, selected_object)
        image = self.fbo.toImage()
        self.fbo.release()
        self.context.doneCurrent()
        return image

    def render(self,
               world: World,
               walls_height: typing.SupportsFloat = 10,
               width: typing.SupportsInt = 640,
               height: typing.SupportsInt = 360,
               selected_object: PhysicalObject | None = None,
               **camera_config: typing.Unpack[CameraConfig]) -> Image:

        im = self.draw(world, walls_height, width, height, selected_object,
                       **camera_config)
        return to_numpy_image(im)

    def save_image(self,
                   world: World,
                   path: str,
                   walls_height: typing.SupportsFloat = 10,
                   width: typing.SupportsInt = 640,
                   height: typing.SupportsInt = 360,
                   selected_object: PhysicalObject | None = None,
                   **camera_config: typing.Unpack[CameraConfig]) -> None:
        im = self.draw(world, walls_height, width, height, selected_object,
                       **camera_config)
        im.save(path)

    def get_position_of_pixel(self, pixel: Pixel) -> Vector3 | None:
        assert self.size
        with self.bind():
            return _get_position_of_pixel(pixel, *self.size, self.camera)


_renderers: weakref.WeakKeyDictionary[
    threading.Thread, OffScreenRenderer] = weakref.WeakKeyDictionary()


def get_renderer() -> OffScreenRenderer:

    # if threading.current_thread() is not threading.main_thread():
    #     raise RuntimeError("Should be called from main thread")

    global _renderers

    t = threading.current_thread()
    if t not in _renderers:
        init()
        _renderers[t] = OffScreenRenderer(share=t is threading.main_thread())
    return _renderers[t]


def render(world: World,
           walls_height: typing.SupportsFloat = 10,
           width: typing.SupportsInt = 640,
           height: typing.SupportsInt = 360,
           selected_object: PhysicalObject | None = None,
           **camera_config: typing.Unpack[CameraConfig]) -> Image:
    return get_renderer().render(world, walls_height, width, height,
                                 selected_object, **camera_config)


def save_image(world: World,
               path: str,
               walls_height: typing.SupportsFloat = 10,
               width: typing.SupportsInt = 640,
               height: typing.SupportsInt = 360,
               selected_object: PhysicalObject | None = None,
               **camera_config: typing.Unpack[CameraConfig]) -> None:
    return get_renderer().save_image(world, path, walls_height, width, height,
                                     selected_object, **camera_config)


def get_position_of_pixel(pixel: Pixel) -> Vector3 | None:
    return get_renderer().get_position_of_pixel(pixel)

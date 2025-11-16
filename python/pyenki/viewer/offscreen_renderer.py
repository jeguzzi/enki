from __future__ import annotations

import threading
import typing

from PySide6.QtGui import (QImage, QOffscreenSurface, QOpenGLContext,
                           QSurfaceFormat)
from PySide6.QtOpenGL import (QOpenGLFramebufferObject,
                              QOpenGLFramebufferObjectFormat)

from .. import Image, VectorLike, World
from .camera import Camera
from .renderer import Renderer
from .utils import init, to_3d, to_numpy_image
import weakref


class OffScreenRenderer:

    def __init__(self, share: bool = False) -> None:
        # Add a check that we initialized
        if threading.current_thread() is not threading.main_thread():
            share = False
        self._thread_id = threading.current_thread().native_id
        self.context: QOpenGLContext | None = QOpenGLContext()
        if share:
            self.context.setShareContext(QOpenGLContext.globalShareContext())
        self.context.setFormat(QSurfaceFormat.defaultFormat())
        self.context.create()
        if not self.context.isValid():
            raise RuntimeError("Unable to create context")
        self.surface = QOffscreenSurface()
        self.surface.setFormat(QSurfaceFormat.defaultFormat())
        self.surface.create()
        if not self.surface.isValid():
            raise RuntimeError("Unable to create offscreen surface")
        self.renderer = Renderer.get(self.context)
        self.context.destroyed.connect(self.context_destroyed)
        # self.context.aboutToBeDestroyed.connect(self.context_will_destroy)

    def context_destroyed(self) -> None:
        self.context = None

    def __del__(self) -> None:
        print('OffScreenRenderer.__del__')
        if self._thread_id == threading.current_thread().native_id and self.context:
            self.context.makeCurrent(self.surface)
            self.renderer.context_will_be_destroyed(self.context)
            self.context.doneCurrent()

    def draw(self,
             world: World,
             camera_reset: bool = False,
             camera_position: VectorLike = (0, 0),
             camera_altitude: typing.SupportsFloat = 0,
             camera_yaw: typing.SupportsFloat = 0,
             camera_pitch: typing.SupportsFloat = 0,
             camera_is_ortho: bool = False,
             walls_height: typing.SupportsFloat = 10,
             width: typing.SupportsInt = 640,
             height: typing.SupportsInt = 360) -> QImage:
        assert self.context
        assert self._thread_id == threading.current_thread().native_id
        width = int(width)
        height = int(height)
        self.context.makeCurrent(self.surface)
        fbo_format = QOpenGLFramebufferObjectFormat()
        fbo_format.setAttachment(
            QOpenGLFramebufferObject.Attachment.CombinedDepthStencil)
        fbo = QOpenGLFramebufferObject(width, height, fbo_format)
        self.context.functions().glViewport(0, 0, width, height)
        fbo.bind()
        camera = Camera()
        camera.set_viewport(width, height)
        camera.is_ortho = camera_is_ortho
        if camera_reset:
            camera.reset(world)
        else:
            camera.position = to_3d(camera_position, camera_altitude)
            camera.yaw = float(camera_yaw)
            camera.pitch = float(camera_pitch)
        self.renderer.draw(world, float(walls_height), camera.matrix,
                           camera.projection)
        image = fbo.toImage()
        fbo.release()
        self.context.doneCurrent()
        return image

    def render(self,
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

        im = self.draw(world, camera_reset, camera_position, camera_altitude,
                       camera_yaw, camera_pitch, camera_is_ortho, walls_height,
                       width, height)
        return to_numpy_image(im)

    def save_image(self,
                   world: World,
                   path: str,
                   camera_reset: bool = False,
                   camera_position: VectorLike = (0, 0),
                   camera_altitude: typing.SupportsFloat = 0,
                   camera_yaw: typing.SupportsFloat = 0,
                   camera_pitch: typing.SupportsFloat = 0,
                   camera_is_ortho: bool = False,
                   walls_height: typing.SupportsFloat = 10,
                   width: typing.SupportsInt = 640,
                   height: typing.SupportsInt = 360) -> None:
        im = self.draw(world, camera_reset, camera_position, camera_altitude,
                       camera_yaw, camera_pitch, camera_is_ortho, walls_height,
                       width, height)
        im.save(path)


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
           camera_reset: bool = False,
           camera_position: VectorLike = (0, 0),
           camera_altitude: typing.SupportsFloat = 0,
           camera_yaw: typing.SupportsFloat = 0,
           camera_pitch: typing.SupportsFloat = 0,
           camera_is_ortho: bool = False,
           walls_height: typing.SupportsFloat = 10,
           width: typing.SupportsInt = 640,
           height: typing.SupportsInt = 360) -> Image:
    return get_renderer().render(world, camera_reset, camera_position,
                                 camera_altitude, camera_yaw, camera_pitch,
                                 camera_is_ortho, walls_height, width, height)


def save_image(world: World,
               path: str,
               camera_reset: bool = False,
               camera_position: VectorLike = (0, 0),
               camera_altitude: typing.SupportsFloat = 0,
               camera_yaw: typing.SupportsFloat = 0,
               camera_pitch: typing.SupportsFloat = 0,
               camera_is_ortho: bool = False,
               walls_height: typing.SupportsFloat = 10,
               width: typing.SupportsInt = 640,
               height: typing.SupportsInt = 360) -> None:
    return get_renderer().save_image(world, path, camera_reset,
                                     camera_position, camera_altitude,
                                     camera_yaw, camera_pitch, camera_is_ortho,
                                     walls_height, width, height)

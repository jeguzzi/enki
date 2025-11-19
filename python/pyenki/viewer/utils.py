import typing

import numpy as np
import numpy.typing
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtCore import QCoreApplication, QEventLoop, Qt, QTimer
from PySide6.QtGui import (QImage, QOpenGLContext, QOpenGLFunctions,
                           QSurfaceFormat, QVector4D)
from PySide6.QtWidgets import QApplication

from .. import Image, PhysicalObject, VectorLike, World
from .camera import Camera
from .renderer import Renderer
from .types import Pixel, Vector3


def init(share: bool = True) -> None:
    if not QApplication.instanceExists():
        if share:
            QCoreApplication.setAttribute(
                Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
        # QCoreApplication.setAttribute(
        #     Qt.ApplicationAttribute.AA_DontCheckOpenGLContextThreadAffinity)
        _ = QApplication([])
        setup_context()


def run(duration: typing.SupportsFloat = -1) -> None:
    app = QApplication.instance()
    if app:
        duration = float(duration)
        if duration >= 0:
            loop = QEventLoop()
            QTimer.singleShot(int(1000 * duration), loop, loop.quit)
            loop.exec()
        else:
            app.exec()


def cleanup() -> None:
    Renderer.cleanup()


def to_numpy_image(image: QImage) -> Image:
    image = image.convertToFormat(QImage.Format.Format_RGB888)
    buffer = image.bits()
    shape = (image.height(), image.width(), 3)
    return np.copy(np.frombuffer(buffer, np.uint8).reshape(shape))


def setup_context() -> None:
    fmt = QSurfaceFormat()
    fmt.setVersion(4, 0)
    fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CoreProfile)
    QSurfaceFormat.setDefaultFormat(fmt)


def print_gl() -> None:
    f = functions()
    print(f"""
Vendor: {f.glGetString(GL.GL_VENDOR)}
Renderer: {f.glGetString(GL.GL_RENDERER)}
OpenGL Version: {f.glGetString(GL.GL_VERSION)}
Shader Version: {f.glGetString(GL.GL_SHADING_LANGUAGE_VERSION)}
""")


def functions() -> QOpenGLFunctions:
    return QOpenGLContext.currentContext().functions()


def get_object_at(world: World,
                  position: VectorLike,
                  tolerance: float = 0) -> PhysicalObject | None:
    position = np.asarray(position)
    for obj in world.objects:
        if obj.contains(position, tolerance):
            return obj
    return None


def get_position_of_pixel(pixel: Pixel, width: int, height: int,
                          camera: Camera) -> Vector3 | None:
    depth = get_depth_of_pixel(pixel, width, height)
    if depth is None:
        return None
    m = camera.projection * camera.matrix
    m, _ = m.inverted()
    x = (pixel[0] - width * 0.5) / (width * 0.5)
    y = (height - pixel[1] - height * 0.5) / (height * 0.5)
    p = m.map(QVector4D(x, y, 2 * depth - 1, 1))
    if p.w():
        return np.array((p.x(), p.y(), p.z())) / p.w()
    return None


def get_depth_of_pixel(pixel: Pixel, width: int, height: int) -> float | None:
    f = functions()
    data = np.zeros(1, np.float32)
    if pixel[0] < width and pixel[1] < height:
        f.glReadPixels(
            pixel[0],
            height - pixel[1],
            1,
            1,
            GL.GL_DEPTH_COMPONENT,
            GL.GL_FLOAT,
            data.data  # type: ignore
        )
        return float(data[0])
    return None

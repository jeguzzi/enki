from __future__ import annotations

import typing

import numpy as np
import numpy.typing
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtCore import QCoreApplication, QEventLoop, Qt, QTimer
from PySide6.QtGui import QImage, QOpenGLContext, QSurfaceFormat, QVector4D
from PySide6.QtWidgets import QApplication

from .. import Image
from .camera import Camera
from .renderer import Renderer
from .types import Pixel, Vector3

if typing.TYPE_CHECKING:
    from PySide6.QtGui import QOpenGLFunctions


def init(share: bool = True) -> None:
    """
    Initializes the Qt runtime.

    Args:
        share (bool): Whether to share all OpenGL contexts.

    Should be called before creating any py:class:`pyenki.viewer.WorldView`.
    """
    if not QApplication.instanceExists():
        if share:
            QCoreApplication.setAttribute(
                Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
        # QCoreApplication.setAttribute(
        #     Qt.ApplicationAttribute.AA_DontCheckOpenGLContextThreadAffinity)
        _ = QApplication([])
        setup_context()


def run(duration: typing.SupportsFloat = -1) -> None:
    """
    Runs the Qt run-loop for a while.

    Args:
        duration (float): The duration in seconds.
                          Negative values are interpreted as infinite duration.
    """
    app = QApplication.instance()
    if app:
        duration = float(duration)
        if duration >= 0:
            loop = QEventLoop()
            QTimer.singleShot(int(1000 * duration), loop, loop.quit)
            loop.exec()
        else:
            app.exec()


def run_loop(duration: typing.SupportsFloat = -1) -> QEventLoop | QCoreApplication | None:
    """
    Return the Qt run-loop configured for the given duration

    Args:
        duration (float): The duration in seconds.
                          Negative values are interpreted as infinite duration.
    Returns:
        An executable object (an app or a loop) or
        None if the app is not available.
    """
    app = QApplication.instance()
    if app:
        duration = float(duration)
        if duration >= 0:
            loop = QEventLoop()
            QTimer.singleShot(int(1000 * duration), loop, loop.quit)
            return loop
        else:
            return app
    return None


def cleanup() -> None:
    """
    Cleans up the Qt runtime.
    """
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


def get_position_of_pixel(pixel: Pixel, width: int, height: int,
                          camera: Camera) -> Vector3 | None:
    depth = get_depth_of_pixel(pixel, width, height)
    if depth is None:
        return None
    m = camera.projection * camera.matrix
    m, _ = m.inverted()
    x = 2 * pixel[0] - 1
    y = 1 - 2 * pixel[1]
    p = m.map(QVector4D(x, y, 2 * depth - 1, 1))
    if p.w():
        return np.array((p.x(), p.y(), p.z())) / p.w()
    return None


def get_depth_of_pixel(pixel: Pixel, width: int, height: int) -> float | None:
    if pixel[0] < 0 or pixel[1] < 0 or pixel[0] > 1 or pixel[1] > 1:
        return None
    f = functions()
    data = np.zeros(1, np.float32)
    i = round(pixel[0] * (width - 1))
    j = round((1 - pixel[1]) * (height - 1))
    f.glReadPixels(
        i,
        j,
        1,
        1,
        GL.GL_DEPTH_COMPONENT,
        GL.GL_FLOAT,
        data.data  # type: ignore
    )
    return float(data[0])

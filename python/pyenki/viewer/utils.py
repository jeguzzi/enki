import typing

import numpy as np
import numpy.typing
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtCore import QCoreApplication, QEventLoop, Qt, QTimer
from PySide6.QtGui import (QImage, QOpenGLContext, QOpenGLFunctions,
                           QSurfaceFormat)
from PySide6.QtWidgets import QApplication

from .. import Image, VectorLike
from .renderer import Renderer

Vector3: typing.TypeAlias = typing.Annotated[np.typing.NDArray[np.float64],
                                             '[3, 1]']


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


def to_3d(xy: VectorLike, z: typing.SupportsFloat) -> Vector3:
    return np.concatenate([np.asarray(xy), [float(z)]])


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

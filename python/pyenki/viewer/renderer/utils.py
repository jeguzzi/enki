from __future__ import annotations

import ctypes
import pathlib
from collections.abc import Iterator
from contextlib import contextmanager
from typing import Annotated, TypeAlias, cast, TYPE_CHECKING

import numpy as np
from numpy.typing import ArrayLike
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtGui import (QImage, QMatrix4x4, QOffscreenSurface,
                           QOpenGLContext, QSurfaceFormat,
                           QVector3D)
from PySide6.QtOpenGL import (QOpenGLBuffer, QOpenGLShader,
                              QOpenGLShaderProgram, QOpenGLVertexArrayObject)
from shiboken6 import VoidPtr

from ... import Color, DifferentialWheeled, PhysicalObject

if TYPE_CHECKING:
    from PySide6.QtGui import (QOpenGLFunctions)
    from numpy.typing import NDArray

ARGBImageLike: TypeAlias = Annotated[ArrayLike, np.uint8, '[n, m, 4]']


def from_numpy_image(image: ARGBImageLike) -> QImage:
    image = np.asarray(image)
    h, w, c = image.shape
    assert c == 4 and image.dtype == np.uint8
    qimage = QImage(image.data, w, h, c * w, QImage.Format.Format_ARGB32)
    return qimage


def create_vao_with_vertices(
    data: NDArray[np.float32]
) -> tuple[QOpenGLVertexArrayObject, QOpenGLBuffer]:
    vao = QOpenGLVertexArrayObject()
    vbo = QOpenGLBuffer()
    vao.create()
    with QOpenGLVertexArrayObject.Binder(vao):
        vbo.create()
        vbo.bind()
        vbo.allocate(data.data, data.nbytes)  # type: ignore[call-overload]
        setup_vertex_attribs(vbo)
    return vao, vbo


def create_vao_with_indices_and_vertices(
    indices: NDArray[np.float32], vertices: NDArray[np.float32]
) -> tuple[QOpenGLVertexArrayObject, QOpenGLBuffer, QOpenGLBuffer]:
    vao = QOpenGLVertexArrayObject()
    vao.create()
    with QOpenGLVertexArrayObject.Binder(vao):
        ibo = QOpenGLBuffer(QOpenGLBuffer.Type.IndexBuffer)
        ibo.create()
        ibo.bind()
        ibo.allocate(indices.data,
                     indices.nbytes)  # type: ignore[call-overload]
        vbo = QOpenGLBuffer()
        vbo.create()
        vbo.bind()
        vbo.allocate(vertices.data,
                     vertices.nbytes)  # type: ignore[call-overload]
        setup_vertex_attribs(vbo)
    return vao, ibo, vbo


@contextmanager
def switch_context(context: QOpenGLContext | None) -> Iterator[None]:
    if context:
        previous: QOpenGLContext | None = QOpenGLContext.currentContext()
        if previous is context:
            context = None
            previous = None
        else:
            surface = None
            if previous:
                surface = previous.surface()
                previous.doneCurrent()
            if not surface:
                surface = QOffscreenSurface()
                surface.setFormat(QSurfaceFormat.defaultFormat())
                surface.create()
            context.makeCurrent(surface)
    else:
        previous = None
    try:
        yield
    finally:
        if context:
            context.doneCurrent()
        if previous:
            assert surface
            previous.makeCurrent(surface)


def circle(n: int = 100) -> NDArray[np.float64]:
    a = np.linspace(0, 2 * np.pi, n)
    return np.stack(
        [np.cos(a), np.sin(a), np.zeros(len(a))], axis=1, dtype=np.float32)


def to_vector(color: Color) -> QVector3D:
    return QVector3D(*color.components[:3])


def load_numpy_array(resource: pathlib.Path) -> NDArray[np.float64]:
    return cast('NDArray[np.float64]', np.load(resource))


# def load_numpy_array(resource: str) -> NDArray[np.float64]:
#     f = QFile(resource)
#     f.open(QIODevice.OpenModeFlag.ReadOnly)
#     data = f.readAll()
#     rs = np.load(io.BytesIO(data))  # type: ignore[arg-type]
#     f.close()
#     return cast(NDArray[np.float64], rs)


def setup_context() -> None:
    fmt = QSurfaceFormat()
    fmt.setVersion(4, 1)
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


def load_program(vs: str, fs: str) -> QOpenGLShaderProgram:
    shaders = pathlib.Path(__file__).parent / "shaders"
    vertex_shader = shaders / f'{vs}.vs'
    fragment_shader = shaders / f'{fs}.fs'
    program = QOpenGLShaderProgram()
    program.addShaderFromSourceFile(QOpenGLShader.ShaderTypeBit.Vertex,
                                    str(vertex_shader))
    program.addShaderFromSourceFile(QOpenGLShader.ShaderTypeBit.Fragment,
                                    str(fragment_shader))
    program.bindAttributeLocation("vertex", 0)
    program.bindAttributeLocation("texCoord", 1)
    program.bindAttributeLocation("vertex_normal", 2)
    program.link()
    program.bind()
    return program


def setup_program(projection: QMatrix4x4,
                  program: QOpenGLShaderProgram) -> None:
    program.setUniformValue("proj", projection)  # type: ignore[call-overload]
    program.setUniformValue(
        "lightSource.ambient",  # type: ignore[call-overload]
        QVector3D(1.0, 1.0, 1.0) * 0.2)
    program.setUniformValue(
        "lightSource.diffuse",  # type: ignore[call-overload]
        QVector3D(1.0, 1.0, 1.0) * 0.4)
    program.setUniformValue(
        "lightSource.specular",  # type: ignore[call-overload]
        QVector3D(1.0, 1.0, 1.0) * 0.2)
    program.setUniformValue(
        "lightSource.position",  # type: ignore[call-overload]
        QVector3D(1.0, 1.0, 1.0))
    program.setUniformValue(
        "lightModel.ambient",  # type: ignore[call-overload]
        QVector3D(0.0, 0.0, 0.0))
    program.setUniformValue(
        "material.emission",  # type: ignore[call-overload]
        QVector3D(0.0, 0.0, 0.0))
    program.setUniformValue(
        "material.specular",  # type: ignore[call-overload]
        QVector3D(1.0, 1.0, 1.0))
    # HACK ubuntu (bug in latest pyside?)
    loc = program.uniformLocation(b"u_has_texture")
    program.setUniformValue(loc, True)
    # program.setUniformValue("u_has_texture", True)
    loc = program.uniformLocation(b"material.shininess")
    program.setUniformValue(loc, 10.0)
    # program.setUniformValue("material.shininess", 10.0)


def get_transform(obj: PhysicalObject) -> QMatrix4x4:
    m = QMatrix4x4()
    m.translate(*obj.position, 0.0)  # type: ignore[call-overload]
    m.rotate(180 * obj.angle / np.pi, 0, 0, 1)
    return m


def get_wheel_angles(robot: DifferentialWheeled,
                     wheel_radius: float) -> tuple[float, float]:
    return (robot.left_wheel_odometry / wheel_radius * 180 / np.pi,
            robot.right_wheel_odometry / wheel_radius * 180 / np.pi)


def forward_color(color: Color, program: QOpenGLShaderProgram) -> None:
    value = QVector3D(*color.components[:3])
    program.setUniformValue(
        "material.ambient",  # type: ignore[call-overload]
        value)
    program.setUniformValue(
        "material.diffuse",  # type: ignore[call-overload]
        value)


def forward_transform(transform: QMatrix4x4,
                      program: QOpenGLShaderProgram) -> None:
    program.setUniformValue(
        "matrix",  # type: ignore[call-overload]
        transform)
    program.setUniformValue(
        "u_normal_matrix",  # type: ignore[call-overload]
        transform.normalMatrix())


def enable_texture(value: bool, program: QOpenGLShaderProgram) -> None:
    program.setUniformValue(
        "u_has_texture",  # type: ignore[call-overload]
        value)


def setup_vertex_attribs(vbo: QOpenGLBuffer, texture: bool = True) -> None:
    f = functions()
    float_size = ctypes.sizeof(ctypes.c_float)
    vbo.bind()
    if texture:
        f.glEnableVertexAttribArray(1)
        stride = 8
        normals = 5
        f.glVertexAttribPointer(
            1,
            2,
            int(GL.GL_FLOAT),
            int(GL.GL_FALSE),
            stride * float_size,
            VoidPtr(3 * float_size)  # type: ignore
        )
    else:
        stride = 5
        normals = 3
    f.glEnableVertexAttribArray(0)
    f.glVertexAttribPointer(
        0,
        3,
        int(GL.GL_FLOAT),
        int(GL.GL_FALSE),
        stride * float_size,
        VoidPtr(0)  # type: ignore
    )
    f.glEnableVertexAttribArray(2)
    f.glVertexAttribPointer(
        2,
        3,
        int(GL.GL_FLOAT),
        int(GL.GL_FALSE),
        stride * float_size,
        VoidPtr(normals * float_size)  # type: ignore
    )
    vbo.release()


def functions() -> QOpenGLFunctions:
    return QOpenGLContext.currentContext().functions()

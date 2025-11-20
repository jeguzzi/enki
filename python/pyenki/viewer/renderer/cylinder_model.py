from __future__ import annotations

import numpy as np
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtOpenGL import (QOpenGLVertexArrayObject)

from ... import PhysicalObject
from .utils import (functions, circle, create_vao_with_vertices, enable_texture,
                    forward_transform, get_transform, switch_context)
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from PySide6.QtOpenGL import (QOpenGLBuffer, QOpenGLShaderProgram)
    from PySide6.QtGui import QMatrix4x4, QOpenGLContext


def cylinder_data(n: int = 60) -> np.ndarray:
    ps = circle(n=n)
    delta = np.array([0, 0, 1], dtype=np.float32)
    top = ps + delta
    vertices = np.concatenate([ps, top, [delta], top])
    normals = np.concatenate([ps, ps, [delta] * (len(ps) + 1)])
    # TRIANGLE_STRIP
    ixs = np.arange(0, n, 1)
    side_indices = np.dstack([ixs + n, ixs]).flatten()
    # TRIANGLE_FAN
    top_indices = np.arange(2 * n, 3 * n + 1, 1)
    textures = np.zeros((*vertices.shape[:-1], 2), dtype=np.float32)
    # print(vertices.shape, textures.shape)
    indices = np.concatenate([side_indices, top_indices])
    data = np.concatenate([vertices, textures, normals],
                          axis=1).reshape(-1, 8)[indices]
    return data  # type: ignore


class CylinderModel:

    def __init__(self) -> None:
        self.vbo: QOpenGLBuffer | None = None
        self.vao: QOpenGLVertexArrayObject | None = None
        self.N = 60

    def init(self, ctx: QOpenGLContext) -> None:
        with switch_context(ctx):
            self.vao, self.vbo = create_vao_with_vertices(
                cylinder_data(n=self.N))

    def destroy(self) -> None:
        if self.vbo:
            self.vbo.destroy()
        self.vbo = None
        if self.vao:
            self.vao.destroy()
        self.vao = None

    def draw(self, obj: PhysicalObject, program: QOpenGLShaderProgram,
             camera: QMatrix4x4, projection: QMatrix4x4,
             ctx: QOpenGLContext) -> None:
        if self.vao is None:
            self.init(ctx)
        assert self.vao is not None
        f = functions()
        t = get_transform(obj)
        t.scale(obj.radius, obj.radius, obj.height)
        forward_transform(camera * t, program)
        enable_texture(False, program)
        with QOpenGLVertexArrayObject.Binder(self.vao):
            f.glDrawArrays(GL.GL_TRIANGLE_STRIP, 0, 2 * self.N)
            f.glDrawArrays(GL.GL_TRIANGLE_FAN, 2 * self.N, self.N + 1)

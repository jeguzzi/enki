from __future__ import annotations

import numpy as np
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtGui import QMatrix4x4
from PySide6.QtOpenGL import (QOpenGLVertexArrayObject)

from ... import World
from .utils import (create_vao_with_vertices, forward_transform, functions,
                    switch_context)
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from PySide6.QtOpenGL import (QOpenGLBuffer, QOpenGLShaderProgram)
    from PySide6.QtGui import QOpenGLContext
    from numpy.typing import NDArray


def flat_world_data() -> NDArray[np.float32]:
    vertices = np.array(
        [[-0.5, -0.5, 0], [0.5, -0.5, 0], [-0.5, 0.5, 0], [0.5, 0.5, 0]],
        dtype=np.float32)
    n = np.array([0, 0, 1], dtype=np.float32)
    normals = np.array([n] * 4).astype(np.float32)
    textures = np.zeros((4, 2), dtype=np.float32)
    data = np.concatenate([vertices, textures, normals], axis=1).reshape(-1, 8)
    return data  # type: ignore


class FlatWorldModel:

    def __init__(self) -> None:
        self.vbo: QOpenGLBuffer | None = None
        self.vao: QOpenGLVertexArrayObject | None = None

    def destroy(self) -> None:
        if self.vbo:
            self.vbo.destroy()
        self.vbo = None
        if self.vao:
            self.vao.destroy()
        self.vao = None

    def init(self, ctx: QOpenGLContext) -> None:
        with switch_context(ctx):
            self.vao, self.vbo = create_vao_with_vertices(flat_world_data())

    def draw(self, world: World, program: QOpenGLShaderProgram,
             camera: QMatrix4x4, projection: QMatrix4x4,
             ctx: QOpenGLContext) -> None:
        if not self.vao:
            self.init(ctx)
        assert self.vao
        f = functions()
        t = QMatrix4x4()
        t.scale(300, 300, 1)
        forward_transform(camera * t, program)
        with QOpenGLVertexArrayObject.Binder(self.vao):
            f.glDrawArrays(GL.GL_TRIANGLE_STRIP, 0, 4)

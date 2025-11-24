from __future__ import annotations

import numpy as np
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtGui import QMatrix4x4
from PySide6.QtOpenGL import (QOpenGLVertexArrayObject)

from ... import World
from .utils import (create_vao_with_vertices, enable_texture,
                    forward_transform, functions, switch_context)
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from PySide6.QtOpenGL import (QOpenGLBuffer, QOpenGLShaderProgram)
    from PySide6.QtGui import QOpenGLContext
    from numpy.typing import NDArray


def square_world_data() -> NDArray[np.float32]:
    ps = np.array(
        [[-0.5, -0.5, 0], [0.5, -0.5, 0], [0.5, 0.5, 0], [-0.5, 0.5, 0]],
        dtype=np.float32)
    bottom = ps[[0, 1, 2, 3]]
    delta = np.array([0, 0, 1], dtype=np.float32)
    face = np.array([0, 4, 1, 1, 4, 5])
    d = np.array([0, 4, 0, 0, 4, 4])
    faces = np.concatenate([(face + i) % 4 + d for i in range(4)])
    side = np.concatenate([ps, ps + delta], axis=0)[faces]
    rs = np.concatenate([ps, ps[:1]])
    top = rs + delta
    top_far = 5 * rs + delta
    top = np.stack([top, top_far], axis=1).reshape(-1, 3)
    vertices = (np.concatenate([bottom, side, top]).astype(np.float32) +
                np.array([0.5, 0.5, 0], dtype=np.float32))
    e = np.array([0, 0, 1], dtype=np.float32)
    side = 6 * [[0, 1, 0]] + 6 * [[-1, 0, 0]] + 6 * [[0, -1, 0]
                                                     ] + 6 * [[1, 0, 0]]
    normals = np.concatenate([[e] * 4, side, [e] * 10]).astype(np.float32)
    textures_bottom = np.array([[0, 0], [1, 0], [1, 1], [0, 1]],
                               dtype=np.float32)
    textures_face = np.array([[0, 0], [0, 1], [1, 0], [1, 0], [0, 1], [1, 1]],
                             dtype=np.float32)
    textures_top = np.zeros((10, 2), dtype=np.float32)
    # print(textures_bottom.shape, textures_face.shape, textures_top.shape)
    textures = np.concatenate([
        textures_bottom, textures_face, textures_face, textures_face,
        textures_face, textures_top
    ])
    # textures = np.zeros((38, 2), dtype=np.float32)

    data = np.concatenate([vertices, textures, normals], axis=1).reshape(-1, 8)
    return data  # type: ignore


class SquareWorldModel:

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
            self.vao, self.vbo = create_vao_with_vertices(square_world_data())

    def draw(self, world: World, wall_height: float,
             program: QOpenGLShaderProgram, camera: QMatrix4x4,
             projection: QMatrix4x4, ctx: QOpenGLContext) -> None:
        if not self.vao:
            self.init(ctx)
        assert self.vao
        f = functions()
        t = QMatrix4x4()
        t.scale(world.lx, world.ly, wall_height)
        forward_transform(camera * t, program)
        with QOpenGLVertexArrayObject.Binder(self.vao):
            f.glDrawArrays(GL.GL_TRIANGLE_FAN, 0, 4)
            enable_texture(False, program)
            f.glDrawArrays(GL.GL_TRIANGLE_STRIP, 28, 10)
            f.glDrawArrays(GL.GL_TRIANGLES, 4, 24)

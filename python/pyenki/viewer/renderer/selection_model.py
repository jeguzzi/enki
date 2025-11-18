from __future__ import annotations

from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtGui import QImage, QMatrix4x4, QOpenGLContext
from PySide6.QtOpenGL import (QOpenGLBuffer, QOpenGLShaderProgram,
                              QOpenGLVertexArrayObject, QOpenGLTexture)

from ... import PhysicalObject, Color
from .utils import (create_vao_with_vertices, enable_texture,
                    forward_color, forward_transform, functions, get_transform,
                    switch_context)


def data() -> NDArray[np.float32]:
    vertices = np.array(
        [[-1, -1, 0.1], [1, -1, 0.1], [-1, 1, 0.1], [1, 1, 0.1]],
        dtype=np.float32)
    n = np.array([0, 0, 1], dtype=np.float32)
    normals = np.array([n] * 4).astype(np.float32)
    textures = (vertices[:, :2] + 1) / 2
    data = np.concatenate([vertices, textures, normals], axis=1).reshape(-1, 8)
    return data


class SelectionModel:

    def __init__(self) -> None:
        self.vbo: QOpenGLBuffer | None = None
        self.vao: QOpenGLVertexArrayObject | None = None
        self.texture: QOpenGLTexture | None = None

    def destroy(self) -> None:
        if self.vbo:
            self.vbo.destroy()
        self.vbo = None
        if self.vao:
            self.vao.destroy()
        self.vao = None
        if self.texture:
            self.texture.destroy()
        self.texture = None

    def init(self, ctx: QOpenGLContext) -> None:
        with switch_context(ctx):
            root = Path(__file__).parent.parent / "textures"
            self.vao, self.vbo = create_vao_with_vertices(data())
            image = QImage(str(root / "selection.png")).flipped()
            self.texture = QOpenGLTexture(image)

    def draw(self, obj: PhysicalObject, program: QOpenGLShaderProgram,
             camera: QMatrix4x4, projection: QMatrix4x4,
             ctx: QOpenGLContext) -> None:
        if not self.vao:
            self.init(ctx)
        assert self.vao
        assert self.texture
        f = functions()
        t = get_transform(obj)
        t.scale(obj.radius * 1.5, obj.radius * 1.5, 1)
        forward_transform(camera * t, program)
        forward_color(Color.white, program)
        enable_texture(True, program)
        f.glEnable(GL.GL_BLEND)
        f.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
        self.texture.bind()
        with QOpenGLVertexArrayObject.Binder(self.vao):
            f.glDrawArrays(GL.GL_TRIANGLE_STRIP, 0, 4)
        f.glDisable(GL.GL_BLEND)

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtGui import QImage, QOpenGLContext
from PySide6.QtOpenGL import (QOpenGLBuffer, QOpenGLShaderProgram,
                              QOpenGLTexture, QOpenGLVertexArrayObject)

from ... import PhysicalObject, Vector
from .utils import (create_vao_with_vertices, enable_texture, functions,
                    switch_context)

def part_texture(part: PhysicalObject.Part) -> QOpenGLTexture | None:
    colors = sum([[c.components[:3] for c in x] for x in part.textures], [])
    if colors:
        data = (255 * np.array(colors)).astype(np.uint8).reshape(1, -1, 3)
        image = QImage(data.data, data.shape[1], 1, 3 * data.shape[1],
                       QImage.Format.Format_RGB888)
        return QOpenGLTexture(image)
    else:
        return None


def contour(points: list[Vector]) -> NDArray[np.float32]:
    ps = np.concatenate([points, points[:1]], dtype=np.float32)
    ds = np.diff(ps, axis=0)
    if np.cross(ds[0], ds[1]) < 0:
        ps = ps[::-1]
    return ps


def part_data(part: PhysicalObject.Part) -> NDArray[np.float32]:
    ps = contour(part.shape)
    bottom = np.concatenate([ps, [[0]] * len(ps)], axis=1)
    shifted = np.roll(bottom, -1, axis=0)
    bottom2 = np.stack([bottom, shifted], axis=1).reshape(-1, 3)
    delta = np.array([0, 0, part.height], dtype=np.float32)
    top = bottom + delta
    top2 = bottom2 + delta
    # center = np.array([0, 0, 0], dtype=np.float32)
    center = np.mean(bottom, axis=0)
    center_top = center + delta
    vertices = np.concatenate([bottom2, top2, [center_top],
                               top]).astype(np.float32)
    ds = np.diff(ps, axis=0)
    ds = np.concatenate([ds, ds[:1]], axis=0)
    ns = np.stack([ds[:, 1], -ds[:, 0], [0] * len(ds)], axis=1)
    n = np.atleast_2d(np.linalg.norm(ns, axis=1)).T
    ns /= n
    ns = np.stack([ns, ns], axis=1).reshape(-1, 3)
    n = np.array([0, 0, 1], dtype=np.float32)
    ts = np.repeat([n], len(bottom) + 1, axis=0).reshape(-1, 3)
    normals = np.concatenate([ns, ns, ts]).astype(np.float32)
    # TRIANGLES
    m = len(bottom)
    side_indices = np.concatenate([[
        2 * i, 2 * i + 1, 2 * (m + i), 2 * i + 1, 2 * (m + i) + 1, 2 * (m + i)
    ] for i in np.arange(m - 1)])
    # print(side_indices)
    # TRIANGLE_FAN
    top_indices = np.arange(4 * m, 5 * m + 1, 1)
    top_indices = np.concatenate([top_indices, [4 * m + 1]])
    indices = np.concatenate([side_indices, top_indices]).astype(np.uint16)

    lis = [len(x) for x in part.textures]
    if lis:
        ws = np.concatenate([[0], np.cumsum(lis)]) / np.sum(lis)
        e = 0.5 / np.sum(lis)
        xs = np.dstack([ws + e, np.roll(ws, -1) - e]).flatten()
        bottom2_ts = np.dstack([xs, np.zeros(len(xs))]).reshape(-1, 2)
        # bottom_ts = np.dstack([xs, np.zeros(len(xs))]).reshape(-1, 2)
        # bottom2_ts = np.stack([bottom_ts, bottom_ts], axis=1).reshape(-1, 2)
        textures = np.concatenate(
            [bottom2_ts, bottom2_ts,
             np.zeros((len(top) + 1, 2))]).astype(np.float32)
    else:
        textures = np.zeros((*vertices.shape[:-1], 2), dtype=np.float32)
    data = np.concatenate([vertices, textures, normals],
                          axis=1).reshape(-1, 8)[indices]
    return data  # type: ignore


class ObjectPartModel:

    def __init__(self, part: PhysicalObject.Part) -> None:
        self.vbo: QOpenGLBuffer | None = None
        self.vao: QOpenGLVertexArrayObject | None = None
        self.texture: QOpenGLTexture | None = None
        self.part = part
        self.number_of_sides = 0

    def init(self, ctx: QOpenGLContext) -> None:
        with switch_context(ctx):
            self.texture = part_texture(self.part)
            self.vao, self.vbo = create_vao_with_vertices(part_data(self.part))
            self.number_of_sides = len(self.part.shape)

    def destroy(self) -> None:
        # print('ObjectPartModel.destroy')
        if self.vbo:
            self.vbo.destroy()
        self.vbo = None
        if self.vao:
            self.vao.destroy()
        self.vao = None
        if self.texture:
            self.texture.destroy()
        self.texture = None

    def render(self, program: QOpenGLShaderProgram,
               ctx: QOpenGLContext) -> None:
        if not self.vao:
            self.init(ctx)
        assert self.vao
        if self.texture:
            self.texture.bind()
        else:
            enable_texture(False, program)
        f = functions()
        with QOpenGLVertexArrayObject.Binder(self.vao):
            f.glDrawArrays(GL.GL_TRIANGLES, 0, self.number_of_sides * 6)
            enable_texture(False, program)
            f.glDrawArrays(GL.GL_TRIANGLE_FAN, self.number_of_sides * 6,
                           self.number_of_sides + 2)
        # enable_texture(True, program)

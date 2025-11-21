from __future__ import annotations

import threading
import weakref
from typing import Protocol, cast, TYPE_CHECKING

from OpenGL import GL  # type: ignore[import-untyped]
from PySide6.QtGui import (QImage, QColor, QOpenGLFunctions, QSurfaceFormat)
from PySide6.QtOpenGL import QOpenGLTexture

from ... import Color, EPuck, Marxbot, PhysicalObject, Thymio2, World
from .epuck_model import EPuckModel
from .marxbot_model import MarxbotModel
from .object_model import ObjectModel
from .thymio2_model import Thymio2Model
from .utils import (forward_color, forward_transform, load_program,
                    setup_program, switch_context)
from .world_model import WorldModel
from .selection_model import SelectionModel

if TYPE_CHECKING:
    from PySide6.QtGui import (QMatrix4x4, QOpenGLContext, QOpenGLContextGroup)
    from PySide6.QtOpenGL import QOpenGLShaderProgram


class Model(Protocol):

    def __init__(self) -> None:
        ...

    def destroy(self) -> None:
        ...

    def draw(self, object: PhysicalObject, program: QOpenGLShaderProgram,
             camera: QMatrix4x4, projection: QMatrix4x4,
             ctx: QOpenGLContext) -> None:
        ...


# RobotType = TypeVar("RobotType", bound=Robot, covariant=True)
# RobotModels = dict[type[Robot], RobotModel[Robot]]


class Renderer(QOpenGLFunctions):

    ROBOT_MODEL_CLASSES = {
        Marxbot: MarxbotModel,
        EPuck: EPuckModel,
        Thymio2: Thymio2Model,
    }

    _renderers: weakref.WeakKeyDictionary[
        QOpenGLContextGroup, Renderer] = weakref.WeakKeyDictionary()

    @classmethod
    def cleanup(cls) -> None:
        thread_id = threading.current_thread().native_id
        for _ctx, renderer in cls._renderers.items():
            if renderer._thread_id == thread_id:
                renderer.destroy()

    @classmethod
    def get(cls, context: QOpenGLContext) -> Renderer:
        group = context.shareGroup()
        if group not in cls._renderers:
            cls._renderers[group] = Renderer(context)
        r = cls._renderers[group]
        r.contexts.add(context)
        return r

    def __init__(self, context: QOpenGLContext):
        # print('Renderer.__init__')
        self._thread_id = threading.current_thread().native_id
        self._initialized = False
        QOpenGLFunctions.__init__(self)
        self.contexts: set[QOpenGLContext] = set()
        shared_context = context.shareContext()
        if shared_context:
            self._shared_context = weakref.proxy(shared_context)
            # shared_context.aboutToBeDestroyed.connect(self.shared_context_about_to_be_destroyed)
            self._context = None
        else:
            # context.aboutToBeDestroyed.connect(self.context_about_to_be_destroyed)
            self._shared_context = None
            self._context = weakref.proxy(context)
        self._program: QOpenGLShaderProgram | None = None
        self.robot_models: dict[type, Model] = {
            cls: cast('Model', model_cls())
            for cls, model_cls in self.ROBOT_MODEL_CLASSES.items()
        }
        self.world_model = WorldModel()
        self.object_model = ObjectModel()
        self.selection_model = SelectionModel()
        self.default_texture: QOpenGLTexture | None = None

    # def try_to_destroy(self, context: QOpenGLContext | None) -> None:
    #     try:
    #         if self._initialized and context and context.isValid():
    #             self.destroy(context)
    #     except (RuntimeError, ReferenceError):
    #         pass

    # def shared_context_about_to_be_destroyed(self):
    #     print('shared_context_about_to_be_destroyed')
    #     self.try_to_destroy(self._shared_context)

    # def context_about_to_be_destroyed(self):
    #     print('context_about_to_be_destroyed')
    #     self.try_to_destroy(self._context)

    # def __del__(self):
    #     print('__del__')
    #     # try:
    #     #     if self._shared_context:
    #     #         self._shared_context.aboutToBeDestroyed.disconnect(
    #                   self.shared_context_about_to_be_destroyed)
    #     # except ReferenceError:
    #     #     pass
    #     # try:
    #     #     if self._context:
    #     #         self._context.aboutToBeDestroyed.disconnect(self.context_about_to_be_destroyed)
    #     # except ReferenceError:
    #     #     pass
    #     self.try_to_destroy(self._shared_context or self._context)

    def init(self) -> None:
        if self._initialized:
            return
        self.initializeOpenGLFunctions()
        self.glClearColor(0, 0, 0, 1)
        is_core = QSurfaceFormat.defaultFormat().profile(
        ) == QSurfaceFormat.OpenGLContextProfile.CoreProfile
        assert is_core
        self._program = load_program(vs='default', fs='default')
        assert self._program
        self._program.bind()
        # print(self._program.isLinked(), self._program.log())
        i = QImage(1, 1, QImage.Format.Format_RGB888)
        i.fill(QColor(255, 255, 255, 255))
        self.default_texture = QOpenGLTexture(i)
        self._initialized = True

    def remove_world(self, world: World) -> None:
        self.object_model.remove_world(world)

    def add_world(self, world: World) -> None:
        self.object_model.add_world(world)

    def context_will_be_destroyed(self, context: QOpenGLContext) -> None:
        # print('Renderer.context_will_be_destroyed')
        self.contexts.discard(context)
        if not self.contexts:
            self.destroy(context.shareContext())

    def destroy(self, context: QOpenGLContext | None = None) -> None:
        # print('Renderer.destroy', self._initialized)
        if not self._initialized:
            return
        with switch_context(context or self._shared_context or self._context):
            for model in self.robot_models.values():
                model.destroy()
            if self.default_texture:
                self.default_texture.destroy()
            self.default_texture = None
            self.robot_models.clear()
            self.object_model.destroy()
            self.world_model.destroy()
            self.selection_model.destroy()
            if self._program:
                del self._program
            self._program = None
            self._initialized = False

    def draw(self,
             world: World,
             wall_height: float,
             camera: QMatrix4x4,
             proj: QMatrix4x4,
             selected_object: PhysicalObject | None = None) -> None:
        if not self._initialized:
            self.init()
        assert self._program
        assert self.default_texture
        self.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        self.glEnable(GL.GL_DEPTH_TEST)
        self.glEnable(GL.GL_CULL_FACE)
        self._program.bind()
        self.default_texture.bind()
        setup_program(proj, self._program)
        forward_color(Color(1, 1, 1, 1), self._program)
        forward_transform(camera, self._program)
        self.world_model.draw(world,
                              wall_height,
                              camera=camera,
                              program=self._program,
                              projection=proj,
                              ctx=self._shared_context)
        for obj in world.objects:
            model: Model = self.object_model
            for robot_cls, robot_model in self.robot_models.items():
                if isinstance(obj, robot_cls):
                    model = robot_model
                    break
            if model:
                model.draw(obj,
                           camera=camera,
                           program=self._program,
                           projection=proj,
                           ctx=self._shared_context)
        if selected_object:
            self.selection_model.draw(selected_object,
                                      camera=camera,
                                      program=self._program,
                                      projection=proj,
                                      ctx=self._shared_context)
        # log = self._program.log()
        # if log:
        #     print(log)

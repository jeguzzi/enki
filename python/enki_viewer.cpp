/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2016 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication
    arising from research using this software are asked to add the
    following reference:
    Enki - a fast 2D robot simulator
    http://home.gna.org/enki
    Stephane Magnenat <stephane at magnenat dot net>,
    Markus Waibel <markus dot waibel at epfl dot ch>
    Laboratory of Intelligent Systems, EPFL, Lausanne.

    You can redistribute this program and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


    Modified by Jerome Guzzi: ...
*/

#include <Python.h>

#include <pybind11/functional.h>
#include <pybind11/native_enum.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>
#include <stdexcept>

#include "../viewer/Viewer.h"
#include "./enki.h"
#include <QImage>

using namespace Enki;
namespace py = pybind11;

py::array get_rbg_array(const QImage &image) {
  const QImage fb = image.convertToFormat(QImage::Format_RGB888);
  const unsigned char *vs = fb.bits();
  const std::array<ssize_t, 3> shape{fb.height(), fb.width(), 3};
  return py::array(shape, vs);
}

struct PythonViewer : public ViewerWidget {

  PythonViewer(PyWorld *world, double fps = 30, bool updateWorld = true,
               double worldTimeStep = 0, double realTimeFactor = 1,
               bool helpers = true, bool camReset = false,
               Vector camPos = Vector(0.0, 0.0), double camAltitude = 0.0,
               double camYaw = 0.0, double camPitch = 0.0, bool ortho = false,
               double wallsHeight_ = 10.0)
      : ViewerWidget(world, nullptr, (fps > 0) ? int(1000 / fps) : 0,
                     updateWorld, worldTimeStep, realTimeFactor, helpers) {
    cameraIsOrtho = ortho;
    if (camReset) {
      resetCamera();
    } else {
      camera.pos.setX(camPos.x);
      camera.pos.setY(camPos.y);
      camera.altitude = camAltitude;
      camera.yaw = camYaw;
      camera.userYaw = camYaw;
      camera.pitch = camPitch;
    }
    wallsHeight = wallsHeight_;
    setWindowTitle("PyEnki Viewer");
  }

  void timerEvent(QTimerEvent *event) {
    py::gil_scoped_acquire acquire;
    ViewerWidget::timerEvent(event);
  }

  py::array getImage() { return get_rbg_array(grabFramebuffer()); }

  Vector getCameraPosition() const {
    return Vector(camera.pos.x(), camera.pos.y());
  }

  void setCameraPosition(Vector value) {
    camera.pos.setX(value.x);
    camera.pos.setY(value.y);
  }

  // void setWallsHeight(double value) { wallsHeight = value; }

  double getWallsHeight() const { return wallsHeight; }

  void moveCamera(const Vector &targetPosition, double targetAltitude,
                  double targetDistance, std::optional<double> yaw,
                  std::optional<double> pitch) {
    if (yaw) {
      setCameraYaw(*yaw);
    }
    if (cameraIsOrtho) {
      setCameraPosition(targetPosition);
      camera.altitude = targetAltitude + targetDistance;
    } else {

      if (pitch) {
        camera.pitch = *pitch;
      }
      const double x = cos(camera.yaw) * cos(camera.pitch);
      const double y = sin(camera.yaw) * cos(camera.pitch);
      camera.pos.rx() = targetPosition.x - targetDistance * x;
      camera.pos.ry() = targetPosition.y - targetDistance * y;
      camera.altitude = targetAltitude - targetDistance * sin(camera.pitch);
    }
  }

  void pointCamera(const Vector &targetPosition, double targetAltitude,
                   std::optional<Vector> position,
                   std::optional<double> altitude) {
    if (cameraIsOrtho) {
      return;
    }
    if (position) {
      setCameraPosition(*position);
    }
    if (altitude) {
      camera.altitude = *altitude;
    }
    const double x = targetPosition.x - camera.pos.x();
    const double y = targetPosition.y - camera.pos.y();
    const double z = targetAltitude - camera.altitude;
    setCameraYaw(atan2(y, x));
    camera.pitch = atan2(z, sqrt(x * x + y * y));
  }

  double getCameraAltitude() const { return camera.altitude; }

  void setCameraAltitude(double value) { camera.altitude = value; }

  double getCameraYaw() const { return camera.yaw; }

  void setCameraYaw(double value) {
    camera.yaw = value;
    camera.userYaw = value;
  }

  double getCameraPitch() const { return camera.pitch; }

  void setCameraPitch(double value) { camera.pitch = value; }

  py::tuple getCameraPose() const {
    return py::make_tuple(getCameraPosition(), getCameraAltitude(),
                          getCameraYaw(), getCameraPitch());
  }

  void setCameraPose(const py::tuple &value) {
    setCameraPosition(value[0].cast<Vector>());
    setCameraAltitude(value[1].cast<double>());
    setCameraYaw(value[2].cast<double>());
    setCameraPitch(value[3].cast<double>());
  }

  // TODO: add https://doc.qt.io/qtforpython-6/shiboken6/shibokenmodule.html
  // TODO: fix Qt5
  py::object asPyQtWidget() const {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const auto cls =
        py::module_::import("PyQt6.QtOpenGLWidgets").attr("QOpenGLWidget");
    const auto wrapinstance =
        py::module_::import("PyQt6.sip").attr("wrapinstance");
#else
    const auto cls =
        py::module_::import("PyQt5.QtOpenGLWidgets").attr("QOpenGLWidget");
    const auto wrapinstance =
        py::module_::import("PyQt5.sip").attr("wrapinstance");
#endif
    return wrapinstance((long)(this), cls);
  }

  py::object asPySideWidget() const {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const auto cls =
        py::module_::import("PySide6.QtOpenGLWidgets").attr("QOpenGLWidget");
    const auto wrapinstance =
        py::module_::import("shiboken6.Shiboken").attr("wrapInstance");
#else
    const auto cls =
        py::module_::import("PySide2.QtOpenGLWidgets").attr("QOpenGLWidget");
    const auto wrapinstance =
        py::module_::import("shiboken2.Shiboken").attr("wrapinstance");
#endif
    return wrapinstance((long)(this), cls);
  }

  // py::capsule getCapsule() { return py::capsule(this); }
};

void runInViewer(PyWorld *world, double fps = 30, double worldTimeStep = 0,
                 double realTimeFactor = 1, bool helpers = true,
                 bool camReset = false, Vector camPos = Vector(0.0, 0.0),
                 double camAltitude = 0.0, double camYaw = 0.0,
                 double camPitch = 0.0, bool ortho = false,
                 double wallsHeight = 10.0, double duration = -1) {
  EnkiApplication::init();
  PythonViewer viewer(world, fps, true, worldTimeStep, realTimeFactor, helpers,
                      camReset, camPos, camAltitude, camYaw, camPitch, ortho,
                      wallsHeight);
  viewer.setWindowTitle("PyEnki Viewer");
  viewer.show();
  EnkiApplication::run(duration / realTimeFactor);
}

py::array render(PyWorld &world, bool cameraReset = false,
                 Vector camPos = Vector(0, 0), double camAltitude = 0,
                 double camYaw = 0, double camPitch = 0,
                 bool camIsOrtho = false, double wallsHeight = 10,
                 int width = 640, int height = 360) {
  EnkiApplication::init();
  PythonViewer viewer(&world, 0, false, 0, 1, false, cameraReset, camPos,
                      camAltitude, camYaw, camPitch, camIsOrtho, wallsHeight);
  viewer.cameraIsOrtho = camIsOrtho;
  viewer.setFixedWidth(width);
  viewer.setFixedHeight(height);
  return viewer.getImage();
}

void save_image(PyWorld &world, const std::string &path,
                bool cameraReset = false, Vector camPos = Vector(0, 0),
                double camAltitude = 0, double camYaw = 0, double camPitch = 0,
                bool camIsOrtho = false, double wallsHeight = 10,
                double width = 640, double height = 360) {
  EnkiApplication::init();
  PythonViewer viewer(&world, 0, false, 0, 1, false, cameraReset, camPos,
                      camAltitude, camYaw, camPitch, camIsOrtho, wallsHeight);
  viewer.cameraIsOrtho = camIsOrtho;
  viewer.setFixedWidth(width);
  viewer.setFixedHeight(height);
  return viewer.saveImage(path);
}

PYBIND11_MODULE(pyenki_viewer, m) {

  m.def("render", &render, py::arg("world"), py::arg("camera_reset") = false,
        py::arg("camera_position") = Vector(0.0, 0.0),
        py::arg("camera_altitude") = 0.0, py::arg("camera_yaw") = 0.0,
        py::arg("camera_pitch") = 0.0, py::arg("camera_is_ortho") = false,
        py::arg("walls_height") = 10.0, py::arg("width") = 640,
        py::arg("height") = 360, R"doc( 
Render a world to an RGB image array.

Args:
    camera_reset (bool): whether to set the camera in the default pose.
    camera_position (Vector): the horizontal position of the camera.
    camera_altitude (float): the vertical position of the camera.
    camera_yaw (float): the camera rotation around the vertical axis.
    camera_pitch (float): the camera vertical rotation.
    camera_is_ortho (bool): whether the camera uses an orthographic projection.
    walls_height (float): the height of the world boundary in cm.
    width (int): the width of the image in pixels.
    height (int): the height of the image in pixels.

Returns:
    numpy.ndarray[tuple[int, int, int], numpy.dtype[numpy.uint8]]: An array of shape ``(height, width, 3)`` and type ``uint8``.
)doc");
  m.def("save_image", &save_image, py::arg("world"), py::arg("path"),
        py::arg("camera_reset") = false,
        py::arg("camera_position") = Vector(0.0, 0.0),
        py::arg("camera_altitude") = 0.0, py::arg("camera_yaw") = 0.0,
        py::arg("camera_pitch") = 0.0, py::arg("camera_is_ortho") = false,
        py::arg("walls_height") = 10.0, py::arg("width") = 640,
        py::arg("height") = 360, R"doc( 
Render a world to an RGB image array.

Args:
    path (string): The file path where to save the image.
    camera_reset (bool): whether to set the camera in the default pose.
    camera_position (Vector): the horizontal position of the camera.
    camera_altitude (float): the vertical position of the camera.
    camera_yaw (float): the camera rotation around the vertical axis.
    camera_pitch (float): the camera vertical rotation.
    camera_is_ortho (bool): whether the camera uses an orthographic projection.
    walls_height (float): the height of the world boundary in cm.
    width (int): the width of the image in pixels.
    height (int): the height of the image in pixels.

)doc");
  m.def("run_in_viewer", &runInViewer, py::arg("world"), py::arg("fps") = 30,
        py::arg("time_step") = 0, py::arg("factor") = 1,
        py::arg("helpers") = true, py::arg("camera_reset") = false,
        py::arg("camera_position") = Vector(0.0, 0.0),
        py::arg("camera_altitude") = 0.0, py::arg("camera_yaw") = 0.0,
        py::arg("camera_pitch") = 0.0, py::arg("camera_is_ortho") = false,
        py::arg("walls_height") = 10.0, py::arg("duration") = 0,
        py::call_guard<py::gil_scoped_release>(), R"doc( 
Render a world to an RGB image array.

Args:
    fps (float): The framerate of the viewer in frames per second.
    time_step (float): The simulation time step in seconds.
    factor (bool): The real-time factor. If larger than one, the simulation 
                   will run faster then real-time.
    helpers (bool): Whether to display the helpers widgets.
    camera_reset (bool): whether to set the camera in the default pose.
    camera_position (Vector): the horizontal position of the camera.
    camera_altitude (float): the vertical position of the camera.
    camera_yaw (float): the camera rotation around the vertical axis.
    camera_pitch (float): the camera vertical rotation.
    camera_is_ortho (bool): whether the camera uses an orthographic projection.
    walls_height (float): the height of the world boundary in cm.
    duration (float): duration of the simulation in simulated time. 
                      Negative values are interpreted as infinite duration.

)doc");

  py::classh<PythonViewer>(m, "WorldView", R"doc( 
A QOpenGLWidget that displays the world.

Args:
    world (World | None): The world to display.
    fps (float): The framerate of the viewer in frames per second.
    update_world (bool): Whether to trigger world updates before redrawing.
    time_step (float): The simulation time step in seconds.
    factor (bool): The real-time factor. If larger than one, the simulation 
                   will run faster then real-time.
    helpers (bool): Whether to display the helpers widgets.
    camera_reset (bool): whether to set the camera in the default pose.
    camera_position (Vector): the horizontal position of the camera.
    camera_altitude (float): the vertical position of the camera.
    camera_yaw (float): the camera rotation around the vertical axis.
    camera_pitch (float): the camera vertical rotation.
    camera_is_ortho (bool): whether the camera uses an orthographic projection.
    walls_height (float): the height of the world boundary in cm.

Example without PyQt::

    >>> import pyenki
    >>> 
    >>> world = pyenki.World(radius=100)
    >>> epuck = pyenki.EPuck(camera=False)
    >>> epuck.left_wheel_target_speed = 10.0
    >>> epuck.set_led_ring(True)
    >>> world.add_object(epuck)
    >>> # setup Qt: needs to be called before creating the first view
    >>> pyenki.init_ui()
    >>> viewer = pyenki.WorldView(world)
    >>> viewer.show()
    >>> viewer.start_updating_world(0.1)
    >>> # executes the Qt runloop for a while
    >>> pyenki.run_ui(duration=10)

Example with PyQt (composition of two views of the same world)::

    >>> import pyenki
    >>> from PyQt6.QtCore import QCoreApplication, Qt
    >>> from PyQt6.QtWidgets import QWidget, QApplication, QHBoxLayout
    >>> 
    >>> # replaces pyenki.init_ui(): needs to be called before creating any widget
    >>> QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts),
    >>> app = QApplication([])
    >>> 
    >>> world = pyenki.World(radius=100)
    >>> epuck = pyenki.EPuck(camera=False)
    >>> epuck.left_wheel_target_speed = 10.0
    >>> epuck.set_led_ring(True)
    >>> world.add_object(epuck)
    >>> viewer_1 = pyenki.WorldView(world, camera_position=(-20, -20), camera_altitude=20)
    >>> viewer_1.point_camera(target_position=(0, 0), target_altitude=5)
    >>> viewer_2 = pyenki.WorldView(world, helpers=False, camera_altitude=30, camera_is_ortho=True)
    >>> viewer_2.camera_is_ortho = True
    >>> window = QWidget()
    >>> hbox = QHBoxLayout(window)
    >>> window.resize(960, 320)
    >>> hbox.addWidget(viewer_1.widget)
    >>> hbox.addWidget(viewer_2.widget)
    >>> window.show()
    >>> viewer_1.start_updating_world(0.1)
    >>> app.exec()

Attributes:
    world (World | None): the world to display.
    camera_position (Vector): The camera horizontal position.
    camera_altitude (float): The camera vertical position.
    camera_yaw (float): the camera rotation around the vertical axis.
    camera_pitch (float): the camera vertical rotation.
    camera_pose (tuple[Vector, float, float, float]): the camera pose as ``(position, altitude, yaw, pitch)``.
    camera_is_ortho (bool): whether the camera uses an orthographic projection.
    walls_height (float): the height of the world boundary in cm (readonly).
    tracking (bool): whether tracking is active.
    helpers (bool): whether to display the helpers widgets.
    image (numpy.ndarray[tuple[int, int, int], numpy.dtype[numpy.uint8]]): the currently rendered image.
    widget (QOpenGLWidget): this view sip-wrapped so to be manipulable by PyQt.
)doc")
      .def(py::init<PyWorld *, double, bool, double, double, bool, bool, Vector,
                    double, double, double, bool, double>(),
           py::arg("world") = py::none(), py::arg("fps") = 30,
           py::arg("update_world") = false, py::arg("time_step") = 0,
           py::arg("factor") = 1, py::arg("helpers") = true,
           py::arg("camera_reset") = false,
           py::arg("camera_position") = Vector(0.0, 0.0),
           py::arg("camera_altitude") = 0.0, py::arg("camera_yaw") = 0.0,
           py::arg("camera_pitch") = 0.0, py::arg("camera_is_ortho") = false,
           py::arg("walls_height") = 10.0)
      .def("show", &PythonViewer::show, R"doc( 
Shows the view
)doc")
      .def("hide", &PythonViewer::hide, R"doc( 
Hide the view
)doc")
      .def("reset_camera", &PythonViewer::resetCamera, R"doc( 
Reset the camera pose
)doc")
      .def("start_updating_world", &PythonViewer::startUpdatingWorld,
           py::arg("time_step") = 0, py::arg("factor") = 1, R"doc( 
Start updating the world before redrawing the view.

Args:
    time_step (float): The simulation time step in seconds.
    factor (bool): The real-time factor. If larger than one, the simulation 
                   will run faster then real-time.
)doc")
      .def("stop_updating_world", &PythonViewer::stopUpdatingWorld, R"doc( 
Stop updating the world before redrawing the view.
)doc")
      .def("move_camera", &PythonViewer::moveCamera, py::arg("target_position"),
           py::arg("target_altitude") = 0, py::arg("target_distance") = 30,
           py::arg("yaw") = py::none(), py::arg("pitch") = py::none(), R"doc( 
Move the camera so to point towards the target.

Args:
    target_position (Vector): The target horizontal position in cm.
    target_altitude (float): The target vertical position in cm.
    target_distance (float): The distance to the target.
    yaw (float | None): Optionally sets the camera yaw.
    pitch (float | None): Optionally sets the camera pitch.
)doc")
      .def("point_camera", &PythonViewer::pointCamera,
           py::arg("target_position"), py::arg("target_altitude") = 0,
           py::arg("position") = py::none(), py::arg("altitude") = py::none(),
           R"doc( 
Rotate the camera so to point towards the target.

Args:
    target_position (Vector): The target horizontal position in cm.
    target_altitude (float): The target vertical position in cm.
    position (Vector | None): Optionally sets the camera horizontal position in cm.
    altitude (float | None): Optionally sets the camera vertical position in cm.
)doc")
      .def_property("walls_height", &PythonViewer::getWallsHeight, nullptr)
      .def_property("camera_position", &PythonViewer::getCameraPosition,
                    &PythonViewer::setCameraPosition)
      .def_property("camera_altitude", &PythonViewer::getCameraAltitude,
                    &PythonViewer::setCameraAltitude)
      .def_property("camera_yaw", &PythonViewer::getCameraYaw,
                    &PythonViewer::setCameraYaw)
      .def_property("camera_pitch", &PythonViewer::getCameraPitch,
                    &PythonViewer::setCameraPitch)
      .def_property("camera_pose", &PythonViewer::getCameraPose,
                    &PythonViewer::setCameraPose)
      .def_property("world", &PythonViewer::getWorld,
                    [](PythonViewer &v, PyWorld *world) { v.setWorld(world); })
      .def_readwrite("camera_is_ortho", &PythonViewer::cameraIsOrtho)
      .def_property("tracking", &PythonViewer::isTrackingActivated,
                    &PythonViewer::setTracking)
      .def_readwrite("helpers", &PythonViewer::displayHelpers)
      .def_property("image", &PythonViewer::getImage, nullptr)
      .def_property("pyside_widget", &PythonViewer::asPySideWidget, nullptr)
      .def_property("pyqt_widget", &PythonViewer::asPyQtWidget, nullptr)
      .def("save_image", &PythonViewer::saveImage, R"doc( 
Save the image to a file.

Args:
    path (string): file path where to save the image.
)doc");

  m.def("init", &EnkiApplication::init, py::arg("share") = true, R"doc( 
Initialize the Qt runtime.

Args:
    share (bool): Whether to share all OpenGL contexts.

Should be called before creating any py:class:`WorldView`.
)doc");
  m.def("run", &EnkiApplication::run, py::arg("duration") = -1,
        py::call_guard<py::gil_scoped_release>(), R"doc( 
Run the Qt run-loop for a while.

Args:
    duration (float): The duration in seconds. 
                      Negative values are interpreted as infinite duration.
)doc");
  m.def("cleanup", &EnkiApplication::cleanup, R"doc( 
Cleanup the Qt runtime.
)doc");
}

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
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>

#include "../enki/Geometry.h"
#include "../enki/PhysicalEngine.h"
#include "../enki/Types.h"
#include "../enki/robots/e-puck/EPuck.h"
#include "../enki/robots/marxbot/Marxbot.h"
#include "../enki/robots/thymio2/Thymio2.h"
#include "../viewer/Viewer.h"
#include <QImage>

using namespace Enki;
namespace py = pybind11;

class PyWorld;

using Termination = std::function<bool(const PyWorld &)>;
using Callback = std::function<void(PyWorld &)>;

namespace pybind11 {
namespace detail {
template <> struct type_caster<Vector> {

  PYBIND11_TYPE_CASTER(Vector, const_name("Vector"));

  static handle cast(const Vector &src, return_value_policy policy,
                     handle parent) {
    py::array_t<double> ts(2);
    py::buffer_info buf = ts.request();
#if 1
    double *ds = static_cast<double *>(buf.ptr);
    ds[0] = src.x;
    ds[1] = src.y;
#else
    buf.ptr = (void *)(&(src.x));
#endif
    return type_caster<py::array_t<double>>::cast(&ts, policy, parent);
  }

  bool load(handle src, bool convert) {

    if (isinstance<sequence>(src)) {
      const auto seq = reinterpret_borrow<sequence>(src);
      if (seq.size() != 2) {
        return false;
      }
      value.x = seq[0].cast<double>();
      value.y = seq[1].cast<double>();
      return true;
    }
    if (isinstance<array>(src)) {
      auto args = reinterpret_borrow<py::array_t<double>>(src);
      // tuple args(src, true);
      if (args.size() != 2)
        return false;
      value.x = args.data()[0];
      value.y = args.data()[1];
      return true;
    }
    return false;
  }
};

#if CONVERT_COLOR
template <> struct type_caster<Color> {

  PYBIND11_TYPE_CASTER(Color, const_name("Color"));

  static handle cast(const Color &src, return_value_policy policy,
                     handle parent) {
    std::tuple<double, double, double, double> ts{
        src.components[0], src.components[1], src.components[2],
        src.components[3]};
    return type_caster<std::tuple<double, double, double, double>>::cast(
        &ts, policy, parent);
  }

  bool load(handle src, bool convert) {
    if (!src || src.is_none())
      return false;
    auto args = reinterpret_borrow<tuple>(src);
    // tuple args(src, true);
    if (len(args) != 4)
      return false;
    for (int i = 0; i < 4; ++i) {
      value.components[i] = args[i].cast<double>();
    }
    return true;
  }
};
#endif

} // namespace detail
} // namespace pybind11

// wrappers for world

py::tuple getColorComponents(const Color &color) {
  return py::make_tuple(color.components[0], color.components[1],
                        color.components[2], color.components[3]);
}

void setColorComponents(Color &color, py::tuple values) {
  if (len(values) != 4)
    throw std::runtime_error(
        "Tuple used to set components must be of length 4");
  color.components[0] = values[0].cast<double>();
  color.components[1] = values[1].cast<double>();
  color.components[2] = values[2].cast<double>();
  color.components[3] = values[3].cast<double>();
}

static World::GroundTexture loadTexture(const std::string &fileName) {
  /*World::GroundTexture t;

  std::ifstream ifs(ppmFileName.c_str(), std::ifstream::in);
  if (!ifs.good())
          throw std::runtime_error("Cannot open file " + ppmFileName);
  std::string magic;
  ifs >> magic;
  if (magic != "P3")
          throw std::runtime_error("Not a PPM file: " + ppmFileName);
  ifs >> t.width;
  ifs >> t.height;
  int valuesScale;
  ifs >> valuesScale;
  t.data.reserve(t.width*t.height);
  for (int y = 0; y < t.height; ++y)
  {
          for (int x = 0; x < t.width; ++x)
          {
                  unsigned r, g, b;
                  ifs >> r >> g >> b;
                  if (ifs.eof())
                          throw std::runtime_error("Early end-of-file: " +
  ppmFileName); r = (r * 255) / valuesScale; g = (g * 255) / valuesScale; b = (b
  * 255) / valuesScale; t.data.push_back(r|(g<<8)|(b<<16));
          }
  }

  return t;*/
  QImage image(fileName.c_str());
  QImage gt(image.convertToFormat(QImage::Format_ARGB32));

#if QT_VERSION >= QT_VERSION_CHECK(4, 7, 0)
  return World::GroundTexture(gt.width(), gt.height(),
                              (const uint32_t *)gt.constBits());
#else
  return World::GroundTexture(gt.width(), gt.height(), (uint32_t *)gt.bits());
#endif
}

// wrappers for robots

#define OVERRIDE_CONTROL_STEP(cname, dname)                                    \
public:                                                                        \
  void controlStep(double dt) override {                                       \
    control_step(dt);                                                          \
    cname::controlStep(dt);                                                    \
  }                                                                            \
                                                                               \
private:                                                                       \
  void control_step(double dt) {                                               \
    PYBIND11_OVERRIDE_IMPL(PYBIND11_TYPE(void), PYBIND11_TYPE(dname),          \
                           "controlStep", dt);                                 \
  }

struct PyPhysicalObject : public PhysicalObject {
  using PhysicalObject::PhysicalObject;
  OVERRIDE_CONTROL_STEP(PhysicalObject, PyPhysicalObject)
};

struct PyMarxbot : public Marxbot {
  using Marxbot::Marxbot;
  OVERRIDE_CONTROL_STEP(Marxbot, PyMarxbot)
};

struct PyEPuck : public EPuck {
  using EPuck::EPuck;
  OVERRIDE_CONTROL_STEP(EPuck, PyEPuck)
};

struct PyThymio2 : public Thymio2 {
  using Thymio2::Thymio2;
  OVERRIDE_CONTROL_STEP(Thymio2, PyThymio2)
};

void set_thymio_rgb_led(Thymio2 &thymio, Thymio2::LedIndex index, double red,
                        double green, double blue) {
  const double i = std::max(std::max(red, green), blue);
  if (i) {
    red = std::clamp<double>(red / i, 0, 1);
    green = std::clamp<double>(green / i, 0, 1);
    blue = std::clamp<double>(blue / i, 0, 1);
  }
  thymio.setLedColor(index, Color(red, green, blue, i));
}

void set_thymio_rgb_led_i(Thymio2 &thymio, Thymio2::LedIndex index, int red,
                          int green, int blue) {
  set_thymio_rgb_led(thymio, index, red / 31.0, green / 31.0, blue / 31.0);
}

void set_thymio_leds(Thymio2 &thymio, Thymio2::LedIndex first_index, int number,
                     int index, double value) {
  if (index == -1) {
    for (int i = 0; i < number; ++i) {
      thymio.setLedIntensity(Thymio2::LedIndex(first_index + i), value);
    }
  } else if (index < number) {
    thymio.setLedIntensity(Thymio2::LedIndex(first_index + index), value);
  }
}

void set_thymio_leds_i(Thymio2 &thymio, Thymio2::LedIndex first_index,
                       int number, int index, int value) {
  set_thymio_leds(thymio, first_index, number, index, value / 31.0);
}

py::array get_rbg_array(const QImage &image) {
  const QImage fb = image.convertToFormat(QImage::Format_RGB888);
  const unsigned char *vs = fb.bits();
  const std::array<ssize_t, 3> shape{fb.height(), fb.width(), 3};
  return py::array(shape, vs);
}

// void run(World &world, unsigned steps) {
//   for (unsigned i = 0; i < steps; ++i)
//     world.step(1. / 30., 3);
// }

class WorldWithTexturedGround : public World {
  using World::World;
};

struct PyWorld : public World {

  PyWorld(double width, double height, unsigned long seed = 0, const Color &wallsColor = Color::gray,
          const GroundTexture &groundTexture = GroundTexture())
      : World(width, height, seed, wallsColor, groundTexture) {
    takeObjectOwnership = false;
  }

  PyWorld(double radius, unsigned long seed = 0, const Color &wallsColor = Color::gray,
          const GroundTexture &groundTexture = GroundTexture())
      : World(radius, seed, wallsColor, groundTexture) {
    takeObjectOwnership = false;
  }

  PyWorld(unsigned long seed = 0) : World(seed) { takeObjectOwnership = false; }

  void run(unsigned steps = 1, float time_step = 1. / 30.,
           unsigned physics_oversampling = 3,
           const std::optional<Termination> &termination = nullptr,
           const std::optional<Callback> &cb = nullptr) {
    while (true) {
      step(time_step, physics_oversampling);
      if (cb) {
        (*cb)(*this);
      }
      if (std::isfinite(steps)) {
        steps--;
      }
      if (steps == 0 || (termination && (*termination)(*this))) {
        break;
      }
    }
  }
};

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

  py::object asWidget() const {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    const auto cls =
        py::module_::import("PyQt6.QtOpenGLWidgets").attr("QOpenGLWidget");
    const auto wrapinstance =
        py::module_::import("PyQt6.sip").attr("wrapinstance");
#else
    const auto cls =
        py::module_::import("PyQt6.QtOpenGLWidgets").attr("QOpenGLWidget");
    const auto wrapinstance =
        py::module_::import("PyQt6.sip").attr("wrapinstance");
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

Polygon make_polygon(const std::vector<Vector> &ps) {
  Polygon p;
  p.assign(ps.begin(), ps.end());
  return p;
}

Texture make_texture(const Color &color) { return Texture(1, color); }

Textures make_textures(const std::vector<Color> &colors) {
  Textures textures;
  for (const auto &color : colors) {
    textures.push_back(make_texture(color));
  }
  return textures;
}

PhysicalObject::Part _part(const py::tuple &obj) {
  double height = obj[1].cast<double>();
  const auto ps = obj[0].cast<std::vector<Vector>>();
  if (py::len(obj) > 2) {
    auto colors = obj[2].cast<std::vector<Color>>();
    return PhysicalObject::Part(make_polygon(ps), height,
                                make_textures(colors));
  }
  return PhysicalObject::Part(make_polygon(ps), height);
}

py::array render(PyWorld &world, bool cameraReset = false,
                 Vector camPos = Vector(0, 0), double camAltitude = 0,
                 double camYaw = 0, double camPitch = 0,
                 bool camIsOrtho = false, double wallsHeight = 10,
                 double width = 640, double height = 360) {
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

// PYBIND11_MAKE_OPAQUE(Texture)
// PYBIND11_MAKE_OPAQUE(Textures)

PYBIND11_MODULE(pyenki, m) {
  py::options options;
#if PYBIND11_VERSION_MAJOR >= 2 && PYBIND11_VERSION_MINOR >= 10
  py::options.disable_enum_members_docstring();
#endif

  // TODO: complete doc

#if !(CONVERT_COLOR)

  py::class_<Color>(m, "Color", R"doc(
Args:
    r (float): Red channel, in [0, 1], optional (default 0.0)
    g (float): Green channel, in [0, 1], optional (default 0.0)
    b (float): Blue channel, in [0, 1], optional (default 0.0)
    a (float): Alpha channel, in [0, 1], optional (default 1.0)

An RGBA color with values between 0.0 and 1.0.

Attributes:
    black (Color): Black color (readonly)
    gray (Color) : Gray color (readonly)
    white (Color) : White color (readonly)
    red (Color) : Red color (readonly)
    green (Color) : Green color (readonly)
    blue (Color) : Blue color (readonly)
    r (float): Red channel, in [0, 1]
    g (float): Green channel, in [0, 1]
    b (float): Blue channel, in [0, 1]
    a (float): Alpha channel, in [0, 1]
    components (tuple[float, float, float, float]): Components, in [0, 1]

)doc")
      .def(py::init<double, double, double, double>(), py::arg("r") = 0,
           py::arg("g") = 0, py::arg("b") = 0, py::arg("a") = 1)
      .def(py::self += double())
      .def(py::self + double())
      .def(py::self -= double())
      .def(py::self - double())
      .def(py::self *= double())
      .def(py::self * double())
      .def(py::self /= double())
      .def(py::self / double())
      .def(py::self += py::self)
      .def(py::self + py::self)
      .def(py::self -= py::self)
      .def(py::self - py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      // .def("__repr__", &Color::toString)
      .def("__repr__",
           [](const Color &color) {
             py::str s("Color(r=");
             s += py::str(py::cast(color.r()));
             s += py::str(", g=");
             s += py::str(py::cast(color.g()));
             s += py::str(", b=");
             s += py::str(py::cast(color.b()));
             s += py::str(", a=");
             s += py::str(py::cast(color.a()));
             s += py::str(")");
             return s;
           })
      .def("threshold", &Color::threshold, py::arg("limits"), R"doc(
Threshold the color using limit. 
For each component, if value is below limit, set it to 0

Args:
  limit(Color): the threshold

)doc")
      .def("toGray", &Color::toGray, R"doc(
Return the grey level value  

Returns:
  float: the average intensity of the channels.
)doc")
      .def_property_readonly_static(
          "black", [](py::object /* self */) { return Color::black; })
      .def_property_readonly_static(
          "white", [](py::object /* self */) { return Color::white; })
      .def_property_readonly_static(
          "gray", [](py::object /* self */) { return Color::gray; })
      .def_property_readonly_static(
          "red", [](py::object /* self */) { return Color::red; })
      .def_property_readonly_static(
          "green", [](py::object /* self */) { return Color::green; })
      .def_property_readonly_static(
          "blue", [](py::object /* self */) { return Color::blue; })
      .def_property("r", &Color::r, &Color::setR)
      .def_property("g", &Color::g, &Color::setG)
      .def_property("b", &Color::b, &Color::setB)
      .def_property("a", &Color::a, &Color::setA)
      .def_property("components", getColorComponents, setColorComponents);

#endif

  // py::bind_vector<Texture>(m, "Texture");
  // py::bind_vector<Textures>(m, "Textures");

  // Physical objects

  py::class_<PhysicalObject, PyPhysicalObject>(m, "PhysicalObject", R"doc(
The superclass of objects that can be simulated.

Attributes:
    uid (int): A unique identifier (readonly)
    world (World): The world the object belongs to (readonly)
    has_collided (bool): Whether the object has collided 
                         in the last simulation step (readonly)
    name (string): The name (readonly)
    radius (float): The radius of the object's enclosing circle in centimeters (readonly)
    height (float): The object height in centimeters (readonly)
    is_cylindric (bool): True if the object is cylindrical shaped (readonly)
    mass (float): The object mass in kilograms. If below zero, the object is static (readonly)
    moment_of_inertia (float): The object moment of inertial (readonly)

    color (Color): The object color.
    collision_elasticity (float): Elasticity of collisions of this object. 
                                  If 0, soft collision, 100% energy dissipation; if 1, elastic collision, 
                                  0% energy dissipation. Actual elasticity is the product of the elasticity of 
                                  the two colliding objects. Walls are fully elastics
    dry_friction_coefficient (float): The dry friction coefficient mu
    viscous_friction_coefficient (float): The viscous friction coefficient. 
                                          Premultiplied by mass. A value of k applies a force of ``-k * speed * mass``
    viscous_moment_friction_coefficient (float): The viscous friction moment coefficient. 
                                                 Premultiplied by momentOfInertia. A value of k applies a force 
                                                 of ``-k * speed * moment_of_inertia``
    position (Vector) : The position in the world frame in centimeters
    angle (float): The orientation in the world frame in radians
    velocity (Vector): The velocity in the world frame in centimeters per second
    angular_speed (float): The angular speed in the world frame in radians per second
    collision_callback (Callable[[PhysicalObject, PhysicalObject], None] | None): An optional function called when the object perform a control step. 
    control_step_callback (Callable[[PhysicalObject, float], None] | None): An optional function called when the object collides. 
)doc")
      .def_readonly("uid", &PhysicalObject::uid)
      .def_property("world", &PhysicalObject::getWorld, nullptr,
                    py::return_value_policy::reference)
      .def_property(
          "has_collided",
          [](const PhysicalObject &o) { return o.getInterlacedDistance() > 0; },
          nullptr)
      .def_property("name", &PhysicalObject::getName, &PhysicalObject::setName)
      .def_property("radius", &PhysicalObject::getRadius, nullptr)
      .def_property("height", &PhysicalObject::getHeight, nullptr)
      .def_property("is_cylindric", &PhysicalObject::isCylindric, nullptr)
      .def_property("mass", &PhysicalObject::getMass, nullptr)
      .def_property("moment_of_inertia", &PhysicalObject::getMomentOfInertia,
                    nullptr)
      .def_property("_interlaced_distance",
                    &PhysicalObject::getInterlacedDistance, nullptr)
      .def_readwrite("collision_elasticity",
                     &PhysicalObject::collisionElasticity)
      .def_readwrite("dry_friction_coefficient",
                     &PhysicalObject::dryFrictionCoefficient)
      .def_readwrite("viscous_friction_coefficient",
                     &PhysicalObject::viscousFrictionCoefficient)
      .def_readwrite("viscous_moment_friction_coefficient",
                     &PhysicalObject::viscousMomentFrictionCoefficient)
      .def_readwrite("position", &PhysicalObject::pos)
      .def_readwrite("angle", &PhysicalObject::angle)
      .def_readwrite("velocity", &PhysicalObject::speed)
      .def_readwrite("angular_speed", &PhysicalObject::angSpeed)
      .def_property("collision_callback", &PhysicalObject::getCollisionCallback,
                    &PhysicalObject::setCollisionCallback,
                    py::keep_alive<1, 2>())
      .def_property("control_step_callback",
                    &PhysicalObject::getControlCallback,
                    &PhysicalObject::setControlCallback, py::keep_alive<1, 2>())
      .def(
          "control_step",
          [](PyPhysicalObject &o, double dt) { o.controlStep(dt); },
          py::arg("time_step"), R"doc(
The controller associated with the object.

Should be overridden by sub-classes to implement controllers, 
in particular for robots. Alternatively, users can assign a callback
:py:attr:`control_step_callback`.

Arguments:
  time_step (float): The time step of the simulation.

)doc")
      // TODO(OLD): warning setting the "color" property at run time using the
      // viewer from the non-gui thread will lead to a crash because it will do
      // an OpenGL call from that thread
      .def_property("color", &PhysicalObject::getColor,
                    &PhysicalObject::setColor);

  m.def(
      "CircularObject",
      [](double radius, double height, double mass,
         const Color &color = Color()) {
        auto c = std::make_unique<PhysicalObject>();
        c->setCylindric(radius, height, mass);
        c->setColor(color);
        return c;
      },
      py::arg("radius"), py::arg("height"), py::arg("mass"),
      py::arg("color") = Color(), R"doc(
Creates a cylinder.

Arguments:
  radius (float): The radius in cm.
  height (float): The height in cm.
  mass (float): The mass in kg.
  color (Color): The color.
Returns
  PhysicalObject: A cylinder
)doc");

  m.def(
      "RectangularObject",
      [](double l1, double l2, double height, double mass,
         const Color &color = Color()) {
        auto c = std::make_unique<PhysicalObject>();
        c->setRectangular(l1, l2, height, mass);
        c->setColor(color);
        return c;
      },
      py::arg("l1"), py::arg("l2"), py::arg("height"), py::arg("mass"),
      py::arg("color") = Color(), R"doc(
Creates a rectangular prism.

Arguments:
  l1 (float): the side length in cm (x).
  l2 (float): the side length in cm (y).
  height (float): The height in cm.
  mass (float): The mass in kg.
  color (Color): The color.
Returns
  PhysicalObject: A rectangular prism.
)doc");

  m.def(
      "CompositeObject",
      [](py::list &parts, double mass, const Color &color = Color()) {
        auto c = std::make_unique<PhysicalObject>();
        PhysicalObject::Hull hull;
        for (const auto &part : parts) {
          auto p = part.cast<py::tuple>();
          hull += _part(p);
        }
        c->setCustomHull(hull, mass);
        c->setColor(color);
        return c;
      },
      py::arg("parts"), py::arg("mass"), py::arg("color") = Color(), R"doc(
Creates an object composed of parts.

Arguments:
  parts (Sequence[:py:type:`pyenki.Part`]): A sequence of parts.
  mass (float): The mass in kg.
  color (Color): The color.
Returns
  PhysicalObject: A composed object.
)doc");

  m.def(
      "ConvexObject",
      [](const std::vector<Vector> &shape, double height, double mass,
         const Color &color = Color(), const std::vector<Color> &colors = {}) {
        auto c = std::make_unique<PhysicalObject>();
        if (colors.size() == 0) {
          c->setCustomHull(PhysicalObject::Hull(PhysicalObject::Part(
                               make_polygon(shape), height)),
                           mass);
          c->setColor(color);
        } else {
          // TODO(Jerome): is not setting the colors correcly
          c->setCustomHull(
              PhysicalObject::Hull(PhysicalObject::Part(
                  make_polygon(shape), height, make_textures(colors))),
              mass);
        }
        return c;
      },
      py::arg("base"), py::arg("height"), py::arg("mass"),
      py::arg("color") = Color(), py::arg("face_colors") = std::vector<Color>{},
      R"doc(
Creates an vertical prism with a convex polygonal base.

Arguments:
  base (:py:type:`pyenki.Polygon`): 
    The vertices polygonal base in cm. Must be convex.
  height (float): The height in cm.
  mass (float): The mass in kg.
  color (Color): The color.
  face_colors (Sequence[Color]): if not empty, defines the colors of each face.

Returns
  PhysicalObject: A convex prism.
)doc");

  // Robots

  py::class_<Robot, PhysicalObject>(m, "Robot", "Base class for all robots");

  py::class_<DifferentialWheeled, Robot, PhysicalObject>(
      m, "DifferentialWheeled", R"doc(
The virtual base class shared by all robots currently implemented in enki.

Attributes:

    left_wheel_target_speed (float): The target left wheel speed in centimeters per second.
    right_wheel_target_speed (float): The target right wheel speed in centimeters per second.
    left_wheel_encoder_speed (float): The current left wheel speed in centimeters per second (readonly).
    right_wheel_encoder_speed (float): The current right wheel speed in centimeters per second (readonly).
    left_wheel_odometry (float): The left wheel odometry integrated from measured wheel speeds in centimeters (readonly).
    right_wheel_odometry (float): The right wheel odometry integrated from measured wheel speeds in centimeters (readonly).
    wheel_axis (float): The distance between wheels in cm (readonly).
    max_wheel_speed (float): The maximal wheel speed in centimeters per second (readonly).
    wheel_speed_noise (float): The relative noise applied to the target wheel speed at each control step.
)doc")
      .def_readwrite("left_wheel_target_speed", &DifferentialWheeled::leftSpeed)
      .def_readwrite("right_wheel_target_speed",
                     &DifferentialWheeled::rightSpeed)
      .def_readonly("left_wheel_encoder_speed",
                    &DifferentialWheeled::leftEncoder)
      .def_readonly("right_wheel_encoder_speed",
                    &DifferentialWheeled::rightEncoder)
      .def_readonly("left_wheel_odometry", &DifferentialWheeled::leftOdometry)
      .def_readonly("right_wheel_odometry", &DifferentialWheeled::rightOdometry)
      .def_property("wheel_axis", &DifferentialWheeled::getDistBetweenWheels,
                    nullptr)
      .def_property("max_wheel_speed", &DifferentialWheeled::getMaxSpeed,
                    nullptr)
      .def_property("wheel_speed_noise", &DifferentialWheeled::getNoiseAmount,
                    &DifferentialWheeled::setNoiseAmount)
      .def("reset_encoders", &DifferentialWheeled::resetEncoders, R"doc(
Reset the odometry of both wheels.
)doc");

  py::class_<Marxbot, PyMarxbot, DifferentialWheeled, PhysicalObject>(
      m, "Marxbot", R"doc(
        A :py:class:`DifferentialWheeled` Marxbot robot.

        The robot has a planar, omni-directional scanner, placed centrally at a height of 11 cm,
        which detects surrounding objects (distance and color).

        Example::

            >>> import pyenki
            >>> # create a world surrondded by a cylindrical wall.
            >>> world = pyenki.World(r=20.0, walls_color=pyenki.Color(0.8, 0.2, 0.1))
            >>> marxbot = pyenki.Marxbot()
            >>> world.add_object(marxbot)
            >>> marxbot.position = (10.0, 0.0)
            >>> marxbot.angle = 0.0
            >>> # Spin the robot on itself
            >>> marxbot.left_wheel_target_speed = 5.0
            >>> marxbot.right_wheel_target_speed = -5.0
            >>> world.step(0.1)
            >>> # Read the omnidirectional rgbd camera
            >>> # Distances
            >>> marxbot.scanner_distances
            array([30.        , 29.73731892, ...
            >>> # Image
            >>> marxbot.scanner_image
            array([[0.8, 0.2, 0.1], ...

        Attributes:
            scanner_range (float): the range of the scanner. Default is infinite.
            scanner_distances (numpy.ndarray[tuple[int], numpy.dtype[numpy.float64]]): An array of 180 radial distances,
                ordered from -180 degrees to 180 degrees, in centimeters (readonly).
            scanner_image (numpy.ndarray[tuple[int, int], numpy.dtype[numpy.float64]]): An rgba array between 0 and 1 of shape ``(180, 4)`` (readonly).
      )doc")
      .def(py::init<>())
      .def_property(
          "scanner_range",
          [](const Marxbot &robot) {
            return robot.rotatingDistanceSensor.getRange();
          },
          [](Marxbot &robot, double value) {
            robot.rotatingDistanceSensor.setRange(value);
          })
      .def_property(
          "scanner_distances",
          [](const Marxbot &robot) {
            const auto &vs = robot.rotatingDistanceSensor.zbuffer;
            const std::valarray<double> ds = std::sqrt(vs);
            return py::array(static_cast<ssize_t>(ds.size()), &ds[0]);
          },
          nullptr)
      .def_property(
          "scanner_image",
          [](const Marxbot &robot) {
            const auto &vs = robot.rotatingDistanceSensor.image;
            return py::array(static_cast<ssize_t>(vs.size()),
                             &(vs[0].components));
          },
          nullptr);

  py::class_<EPuck, PyEPuck, DifferentialWheeled, PhysicalObject>(m, "EPuck",
                                                                  R"doc(
Args:
  proximity (bool): enable the proximity sensors
  camera (bool): enable the camera
  scanner (bool): enable the scanner

A :py:class:`DifferentialWheeled` e-Puck robot.

The robots has a 60 pixels frontal looking camera placed at a height of 2.2 cm and with a fov of 60 degrees,
and 8 infrared proximity sensors (maximal range 12 cm) placed at a height of 2.5 cm and at angles:
-18, -45, -90, -142, 142, 90, 45, 18 degrees.

Example::

    >>> import pyenki
    >>> world = pyenki.World()
    >>> epuck = pyenki.EPuck()
    >>> world.add_object(epuck)
    >>> epuck.position = (-10, 0)
    >>> epuck.set_led_ring(True)
    >>> world.add_object(pyenki.CircularObject(2.0, 5.0, -1, pyenki.Color(0.3, 0.7, 0)))
    >>> world.step(0.1)
    >>> epuck.prox_values
    array([98.52232283,  3.30197324, ...
    >>> epuck.camera_image
    array([[0.5, 0.5, 0.5, 1. ] ...

Attributes:

    prox_values (numpy.ndarray[tuple[int], numpy.dtype[numpy.float64]]): An array of 8 proximity sensor readings, one for each sensors (readonly).
    prox_distances (numpy.ndarray[tuple[int], numpy.dtype[numpy.float64]]): An array of 8 distances between proximity sensor and nearest obstacles, one for each sensors (readonly).
        please note that this value would *not* directly be accessible by a real robot (readonly).
    scan (numpy.ndarray[tuple[int], numpy.dtype[numpy.float64]]): An array of 64 radial distances, ordered from -180 degrees to 180 degrees, in centimeters.
    camera_image (numpy.ndarray[tuple[int, int], numpy.dtype[numpy.float64]]): An rgba array between 0 and 1 of shape ``(60, 4)`` (readonly).
)doc")
      .def(py::init([](bool proximity = true, bool camera = false,
                       bool scanner = false) {
             unsigned cap = 0;
             if (proximity)
               cap |= EPuck::CAPABILITY_BASIC_SENSORS;
             if (camera)
               cap |= EPuck::CAPABILITY_CAMERA;
             if (scanner)
               cap |= EPuck::CAPABILITY_SCANNER_TURRET;
             return std::make_unique<PyEPuck>(cap);
           }),
           py::arg("proximity") = true, py::arg("camera") = false,
           py::arg("scanner") = false, "")
      .def_property(
          "scan",
          [](const EPuck &robot) {
            const auto &vs = robot.scannerTurret.zbuffer;
            const std::valarray<double> ds = std::sqrt(vs);
            return py::array(static_cast<ssize_t>(ds.size()), &ds[0]);
          },
          nullptr)
      .def_property(
          "prox_distances",
          [](const EPuck &r) {
            const std::vector<double> vs{
                r.infraredSensor0.getDist(), r.infraredSensor1.getDist(),
                r.infraredSensor2.getDist(), r.infraredSensor3.getDist(),
                r.infraredSensor4.getDist(), r.infraredSensor5.getDist(),
                r.infraredSensor6.getDist(), r.infraredSensor7.getDist()};
            return py::array(static_cast<ssize_t>(vs.size()), vs.data());
          },
          nullptr)
      .def_property(
          "prox_values",
          [](const EPuck &r) {
            const std::vector<double> vs{
                r.infraredSensor0.getValue(), r.infraredSensor1.getValue(),
                r.infraredSensor2.getValue(), r.infraredSensor3.getValue(),
                r.infraredSensor4.getValue(), r.infraredSensor5.getValue(),
                r.infraredSensor6.getValue(), r.infraredSensor7.getValue()};
            return py::array(static_cast<ssize_t>(vs.size()), vs.data());
          },
          nullptr)
      .def_property(
          "camera_image",
          [](const EPuck &robot) {
            const auto &vs = robot.camera.image;
            return py::array(static_cast<ssize_t>(vs.size()),
                             &(vs[0].components));
          },
          nullptr)
      .def("set_led_ring", &EPuck::setLedRing, py::arg("value"), R"doc(
Set all the red LEDs on or off.

Args:
  value (bool): the desired LED state.
)doc");

  py::class_<IRCommEvent>(m, "IRCommEvent", R"doc( 
This event is created each time a message is received by at least one proximity sensor.
The sensors that do not receive the message, have the corresponding payloads and intensities set to zero.

Attributes:
    rx_value (int): The received message payload (readonly)
    payloads (numpy.ndarray[tuple[int], numpy.dtype[numpy.int64]]): An array of 7 integer payloads, one for each sensors (readonly).
        The first 5 entries are from frontal sensors ordered from left to right.
        The last two entries are from rear sensors  ordered from left to right.
    intensities (numpy.ndarray[tuple[int], numpy.dtype[numpy.int64]]): An array of 7 integer intensities, one for each sensors (readonly).
        The first 5 entries are from frontal sensors ordered from left to right.
        The last two entries are from rear sensors  ordered from left to right.
)doc")
      .def_property(
          "intensities",
          [](const IRCommEvent &e) {
            const auto &vs = e.intensities;
            return py::array(static_cast<ssize_t>(vs.size()), vs.data());
          },
          nullptr)
      .def_property(
          "payloads",
          [](const IRCommEvent &e) {
            const auto &vs = e.payloads;
            return py::array(static_cast<ssize_t>(vs.size()), vs.data());
          },
          nullptr)
      .def_readonly("rx_value", &IRCommEvent::rx_value);

  py::class_<Thymio2, PyThymio2, DifferentialWheeled, PhysicalObject>(
      m, "Thymio2", R"doc( 
A :py:class:`DifferentialWheeled` Thymio2 robot.
Attribute names mimic the aseba interface, see http://wiki.thymio.org/en:thymioapi.

Example::

    >>> import pyenki
    >>> thymio = pyenki.Thymio2()
    >>> thymio.position = (3.2, 5.6)
    >>> thymio.angle = 1.3
    >>> # Spin the robot on itself
    >>> thymio.motor_left_target = 10.2
    >>> thymio.motor_right_target = -10.2
    >>> # Switch the top LED yellow
    >>> thymio.set_led_top(0.5, 0.5, 0.0)
    >>> thymio.prox_values
    array([   0.        ,    0., ...

For some methods and attributes there is an alternative version with the suffix `_i`
which uses integers in the same units used by aseba. For example, 

- :py:attr:`left_wheel_target_speed_i` uses integers in ``[-500, 500]``, 
  where 500 ticks corresponds to 16.6 cm in :py:attr:`pyenki.DifferentialWheeled.left_wheel_target_speed`

- :py:meth:`set_led_top_i` uses integers in `[0, 31]` where 31 corresponds to full intensity 1.0
  in :py:meth:`set_led_top`.

Attributes:
    prox_values (numpy.ndarray[tuple[int], numpy.dtype[numpy.float64]]): An array of 7 proximity sensor readings, one for each sensors (readonly).
        The first 5 entries are from frontal sensors ordered from left to right.
        The last two entries are from rear sensors  ordered from left to right.
    prox_values_i (numpy.ndarray[tuple[int], numpy.dtype[numpy.int64]]): An array of 7 proximity sensor readings, one for each sensors (readonly).
        The first 5 entries are from frontal sensors ordered from left to right.
        The last two entries are from rear sensors  ordered from left to right.
    prox_distances (numpy.ndarray[tuple[int], numpy.dtype[numpy.float64]]): A list of 7 distances between proximity sensor and nearest obstancle, one for each sensors;
        please note that this value would *not* directly be accessible by a real robot (readonly).
        The first 5 entries are from frontal sensors ordered from left to right.
        The last two entries are from rear sensors  ordered from left to right.
    prox_comm_tx (int): The integer payload to be sent. The real robot can only send 11 bits,
        therefore to be compliant we should limit the value between 0 and 2047.
    prox_comm_enabled (bool): Enable/disable proximity communication.
    prox_comm_events (list[IRCommEvent]): A list of events, one for every received message during the last control step (readonly).
    ground_values (numpy.ndarray[tuple[int], numpy.dtype[numpy.float64]]): An array of 2 ground sensor readings, one for each sensors (readonly)
    ground_values_i (numpy.ndarray[tuple[int], numpy.dtype[numpy.int64]]): An array of 2 ground sensor readings, one for each sensors (readonly)
    left_wheel_target_speed_i (int): The target left wheel speed in ticks per second.
    right_wheel_target_speed_i (int): The target right wheel speed in ticks per second.
    left_wheel_encoder_speed_i (int): The current left wheel speed in ticks per second (readonly).
    right_wheel_encoder_speed_i (int): The current right wheel speed in ticks per second (readonly).
    left_wheel_odometry_i (int): The left wheel odometry integrated from measured wheel speeds in ticks (readonly).
    right_wheel_odometry_i (int): The right wheel odometry integrated from measured wheel speeds in ticks (readonly).
)doc")
      .def(py::init<>(), "TEST")
      .def_property(
          "prox_distances",
          [](const Thymio2 &r) {
            const std::vector<double> vs{
                r.infraredSensor0.getDist(), r.infraredSensor1.getDist(),
                r.infraredSensor2.getDist(), r.infraredSensor3.getDist(),
                r.infraredSensor4.getDist(), r.infraredSensor5.getDist(),
                r.infraredSensor6.getDist()};
            return py::array(static_cast<ssize_t>(vs.size()), vs.data());
          },
          nullptr)
      .def_property(
          "prox_values",
          [](const Thymio2 &r) {
            const std::vector<double> vs{
                r.infraredSensor0.getValue(), r.infraredSensor1.getValue(),
                r.infraredSensor2.getValue(), r.infraredSensor3.getValue(),
                r.infraredSensor4.getValue(), r.infraredSensor5.getValue(),
                r.infraredSensor6.getValue()};
            return py::array(static_cast<ssize_t>(vs.size()), vs.data());
          },
          nullptr)
      .def_property(
          "prox_values_i",
          [](const Thymio2 &r) {
            const std::vector<int> vs{
                static_cast<int>(r.infraredSensor0.getValue()),
                static_cast<int>(r.infraredSensor1.getValue()),
                static_cast<int>(r.infraredSensor2.getValue()),
                static_cast<int>(r.infraredSensor3.getValue()),
                static_cast<int>(r.infraredSensor4.getValue()),
                static_cast<int>(r.infraredSensor5.getValue()),
                static_cast<int>(r.infraredSensor6.getValue())};
            return py::array(static_cast<ssize_t>(vs.size()), vs.data());
          },
          nullptr)
      .def_property(
          "ground_values",
          [](const Thymio2 &r) {
            const std::vector<double> vs{r.groundSensor0.getValue(),
                                         r.groundSensor0.getValue()};
            return py::array(static_cast<ssize_t>(vs.size()), vs.data());
          },
          nullptr)
      .def_property(
          "ground_values_i",
          [](const Thymio2 &r) {
            const std::vector<int> vs{
                static_cast<int>(r.groundSensor0.getValue()),
                static_cast<int>(r.groundSensor0.getValue())};
            return py::array(static_cast<ssize_t>(vs.size()), vs.data());
          },
          nullptr)
      .def_property(
          "prox_comm_tx", [](const Thymio2 &r) { return r.irComm.get_tx(); },
          [](Thymio2 &r, int value) { r.irComm.set_tx(value); })
      .def_property(
          "prox_comm_enabled",
          [](const Thymio2 &r) { return r.irComm.get_enable(); },
          [](Thymio2 &r, bool value) { r.irComm.set_enable(value); })
      .def_property(
          "prox_comm_events",
          [](const Thymio2 &r) { return r.irComm.get_events(); }, nullptr)
      .def(
          "set_led_top",
          [](Thymio2 &r, double red = 0, double green = 0, double blue = 0) {
            set_thymio_rgb_led(r, Thymio2::LedIndex::TOP, red, green, blue);
          },
          py::arg("red") = 0, py::arg("green") = 0, py::arg("blue") = 0, R"doc( 
Control the top RGB LED color

Args:
    red (float): the value of the red channel
    green (float): the value of the green channel
    blue (float): the value of the blue channel
)doc")
      .def(
          "set_led_top_i",
          [](Thymio2 &r, int red = 0, int green = 0, int blue = 0) {
            set_thymio_rgb_led_i(r, Thymio2::LedIndex::TOP, red, green, blue);
          },
          py::arg("red") = 0, py::arg("green") = 0, py::arg("blue") = 0, R"doc(
Control the top RGB LED color

Args:
    red (int): the value of the red channel between 0 and 31
    green (int): the value of the green channel between 0 and 31
    blue (int): the value of the blue channel between 0 and 31
)doc")
      .def(
          "set_led_bottom_left",
          [](Thymio2 &r, double red = 0, double green = 0, double blue = 0) {
            set_thymio_rgb_led(r, Thymio2::LedIndex::BOTTOM_LEFT, red, green,
                               blue);
          },
          py::arg("red") = 0, py::arg("green") = 0, py::arg("blue") = 0, R"doc( 
Control the bottom left RGB LED color

Args:
    red (float): the value of the red channel
    green (float): the value of the green channel
    blue (float): the value of the blue channel
)doc")
      .def(
          "set_led_bottom_left_i",
          [](Thymio2 &r, int red = 0, int green = 0, int blue = 0) {
            set_thymio_rgb_led_i(r, Thymio2::LedIndex::BOTTOM_LEFT, red, green,
                                 blue);
          },
          py::arg("red") = 0, py::arg("green") = 0, py::arg("blue") = 0, R"doc(
Control the bottom left RGB LED color

Args:
    red (int): the value of the red channel between 0 and 31
    green (int): the value of the green channel between 0 and 31
    blue (int): the value of the blue channel between 0 and 31
)doc")
      .def(
          "set_led_bottom_right",
          [](Thymio2 &r, double red = 0, double green = 0, double blue = 0) {
            set_thymio_rgb_led(r, Thymio2::LedIndex::BOTTOM_RIGHT, red, green,
                               blue);
          },
          py::arg("red") = 0, py::arg("green") = 0, py::arg("blue") = 0, R"doc( 
Control the bottom right RGB LED color

Args:
    red (float): the value of the red channel
    green (float): the value of the green channel
    blue (float): the value of the blue channel
)doc")
      .def(
          "set_led_bottom_right_i",
          [](Thymio2 &r, int red = 0, int green = 0, int blue = 0) {
            set_thymio_rgb_led_i(r, Thymio2::LedIndex::BOTTOM_RIGHT, red, green,
                                 blue);
          },
          py::arg("red") = 0, py::arg("green") = 0, py::arg("blue") = 0, R"doc(
Control the bottom right RGB LED color

Args:
    red (int): the value of the red channel between 0 and 31
    green (int): the value of the green channel between 0 and 31
    blue (int): the value of the blue channel between 0 and 31
)doc")
      .def(
          "set_led_buttons",
          [](Thymio2 &r, int index, double value) {
            set_thymio_leds(r, Thymio2::LedIndex::BUTTON_UP, 4, index, value);
          },
          py::arg("index"), py::arg("value"), R"doc(
Control the four button LEDs

Args:
    index (int): the index of the LED. Set to -1 to control all LEDs.
    value (float): the desired intensity between 0 and 1.
)doc")
      .def(
          "set_led_buttons_i",
          [](Thymio2 &r, int index, int value) {
            set_thymio_leds_i(r, Thymio2::LedIndex::BUTTON_UP, 4, index, value);
          },
          py::arg("index"), py::arg("value"), R"doc(
Control the four button LEDs

Args:
    index (int): the index of the LED. Set to -1 to control all LEDs.
    value (int): the desired intensity between 0 and 31.
)doc")
      .def(
          "set_led_circle",
          [](Thymio2 &r, int index, double value) {
            set_thymio_leds(r, Thymio2::LedIndex::RING_0, 8, index, value);
          },
          py::arg("index"), py::arg("value"), R"doc(
Control the 8 circle LEDs

Args:
    index (int): the index of the LED. Set to -1 to control all LEDs.
    value (float): the desired intensity between 0 and 1.
)doc")
      .def(
          "set_led_circle_i",
          [](Thymio2 &r, int index, int value) {
            set_thymio_leds_i(r, Thymio2::LedIndex::RING_0, 8, index, value);
          },
          py::arg("index"), py::arg("value"), R"doc(
Control the 8 circle LEDs

Args:
    index (int): the index of the LED. Set to -1 to control all LEDs.
    value (int): the desired intensity between 0 and 31.
)doc")
      .def(
          "set_led_prox",
          [](Thymio2 &r, int index, double value) {
            set_thymio_leds(r, Thymio2::LedIndex::IR_FRONT_0, 8, index, value);
          },
          py::arg("index"), py::arg("value"), R"doc(
Control the 8 proximity LEDs

Args:
    index (int): the index of the LED. Set to -1 to control all LEDs.
    value (float): the desired intensity between 0 and 1.
)doc")
      .def(
          "set_led_prox_i",
          [](Thymio2 &r, int index, int value) {
            set_thymio_leds_i(r, Thymio2::LedIndex::IR_FRONT_0, 8, index,
                              value);
          },
          py::arg("index"), py::arg("value"), R"doc(
Control the 8 proximity LEDs

Args:
    index (int): the index of the LED. Set to -1 to control all LEDs.
    value (int): the desired intensity between 0 and 31.
)doc")
      .def(
          "set_led_left_red",
          [](Thymio2 &r, double value) {
            r.setLedIntensity(Thymio2::LedIndex::LEFT_RED, value);
          },
          py::arg("value"), R"doc(
Control the left red LEDs

Args:
    value (float): the desired intensity between 0 and 1.
)doc")
      .def(
          "set_led_left_red_i",
          [](Thymio2 &r, int value) {
            r.setLedIntensity(Thymio2::LedIndex::LEFT_RED, value / 31.0);
          },
          py::arg("value"), R"doc(
Control the left red LEDs

Args:
    value (int): the desired intensity between 0 and 31.
)doc")
      .def(
          "set_led_left_blue",
          [](Thymio2 &r, double value) {
            r.setLedIntensity(Thymio2::LedIndex::LEFT_BLUE, value);
          },
          py::arg("value"), R"doc(
Control the left blue LEDs

Args:
    value (float): the desired intensity between 0 and 1.
)doc")
      .def(
          "set_led_left_blue_i",
          [](Thymio2 &r, int value) {
            r.setLedIntensity(Thymio2::LedIndex::LEFT_BLUE, value / 31.0);
          },
          py::arg("value"), R"doc(
Control the left blue LEDs

Args:
    value (int): the desired intensity between 0 and 31.
)doc")
      .def(
          "set_led_right_red",
          [](Thymio2 &r, double value) {
            r.setLedIntensity(Thymio2::LedIndex::RIGHT_RED, value);
          },
          py::arg("value"), R"doc(
Control the right red LEDs

Args:
    value (float): the desired intensity between 0 and 1.
)doc")
      .def(
          "set_led_right_red_i",
          [](Thymio2 &r, int value) {
            r.setLedIntensity(Thymio2::LedIndex::RIGHT_RED, value / 31.0);
          },
          py::arg("value"), R"doc(
Control the right red LEDs

Args:
    value (int): the desired intensity between 0 and 31.
)doc")
      .def(
          "set_led_right_blue",
          [](Thymio2 &r, double value) {
            r.setLedIntensity(Thymio2::LedIndex::RIGHT_BLUE, value);
          },
          py::arg("value"), R"doc(
Control the right blue LEDs

Args:
    value (float): the desired intensity between 0 and 1.
)doc")
      .def(
          "set_led_right_blue_i",
          [](Thymio2 &r, int value) {
            r.setLedIntensity(Thymio2::LedIndex::RIGHT_BLUE, value / 31.0);
          },
          py::arg("value"), R"doc(
Control the right blue LEDs

Args:
    value (int): the desired intensity between 0 and 31.
)doc");

  py::class_<PyWorld>(m, "World", R"doc(
The world is the container of all objects and robots.
It is either

- a rectangular arena with walls at all sides::
    
    World(width: float, height: float, seed: int = 0, walls_color: Color = Color.gray)

- a circular area with walls::

    World(width: float, height: float, seed: int = 0, walls_color: Color = Color.gray)

- or an infinite surface::

    World(seed: int = 0)

Args:
    width (float): The rectangular world width in centimeters
    height (float): The rectangular world height in centimeters
    radius (float): The circular world radius in centimeters
    seed (int): The random seed
    walls_color (Color): Optional wall color, default is ``Color.gray``


Example::

    import pyenki

    world = pyenki.World()
    thymio = Thymio2()
    world.add_object(thymio)
    wall = pyenki.RectangularObject(l1=10, l2=50, height=5, mass=1,
                                    color=pyenki.Color(0.5, 0.3, 0.3))
    world.add_object(wall)
    # Run 100 times a 0.1 s long simulation step
    for _ in range(100):
        world.step(0.1)

Attributes:

    objects (list[PhysicalObject]): The list of all objects
    robots (list[Robot]): The list of all robots
    static_objects (list[PhysicalObject]): The list of all objects that are not robots
    control_step_callback (Callable[[World, float], None] | None): A function called at each update step.
    random_seed: The random seed
)doc")
      .def(py::init<unsigned long>(), py::arg("seed") = 0)
      .def(py::init<double, double, unsigned long, const Color &>(), py::arg("width"),
           py::arg("height"), py::arg("seed") = 0, py::arg("walls_color") = Color::gray)
      .def(py::init<double, unsigned long, const Color &>(), py::arg("radius"),
           py::arg("seed") = 0, py::arg("walls_color") = Color::gray)
      .def("step", &World::step, py::arg("time_step"),
           py::arg("physics_oversampling") = 1, R"doc( 
Simulate a timestep

Args:
    time_step (float): the update timestep in seconds, should be below 1 (typically .02-.1)
    physics_oversampling (int): the amount of time the physics is run per step,
                                as usual collisions require a more precise simulation 
                                than the sensor-motor loop frequency
)doc")
      .def("add_object", &World::addObject, py::arg("object"),
           py::keep_alive<1, 2>(), R"doc( 
Add an object to the simulation.

Args:
    object (PhysicalObject): the object to add.
)doc")
      .def("remove_object", &World::removeObject, py::arg("object"), R"doc( 
Remove an object from the simulation.

Args:
    object (PhysicalObject): the object to remove.
)doc")
      // TODO
      .def("copy_random_generator", &World::copyRandom)
      .def_property("random_seed", &World::getRandomSeed, &World::setRandomSeed)
      .def_property("robots", &World::get_robots, nullptr)
      .def_property("static_objects", &World::get_static_objects, nullptr)
      .def_readonly("objects", &World::objects)
      .def_property("control_step_callback", &World::getControlCallback,
                    &World::setControlCallback, py::keep_alive<1, 2>())
      .def("run", &PyWorld::run, py::arg("steps") = 1,
           py::arg("time_step") = 1.0 / 30.0,
           py::arg("physics_oversampling") = 3,
           py::arg("termination") = std::nullopt,
           py::arg("callback") = std::nullopt, R"doc( 
Run a simulation.

Args:
    steps (int): the number of steps.
    time_step (float): the time step.
    physics_oversampling (int): the number of times the physics is updated per step 
                                to get a more fine-grained physical simulation compared to the sensor-motor loop.
    termination (Callable[[World], bool] | None): an optional function that terminates the simulation when it returns True.
    callback (Callable[[World], None] | None): An additional callback executed at each simulation step.
)doc")
      .def("render", &render, py::arg("camera_reset") = false,
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
)doc")
      .def("save_image", &save_image, py::arg("path"),
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

)doc")
      .def("run_in_viewer", &runInViewer, py::arg("fps") = 30,
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

  py::class_<PythonViewer>(m, "WorldView", R"doc( 
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
      .def_property("widget", &PythonViewer::asWidget, nullptr)
      .def("save_image", &PythonViewer::saveImage, R"doc( 
Save the image to a file.

Args:
    path (string): file path where to save the image.
)doc");

  m.def("init_ui", &EnkiApplication::init, R"doc( 
Initialize the Qt runtime.

Should be called before creating any py:class:`WorldView`.
)doc");
  m.def("run_ui", &EnkiApplication::run, py::arg("duration") = -1,
        py::call_guard<py::gil_scoped_release>(), R"doc( 
Run the Qt run-loop for a while.

Args:
    duration (float): The duration in seconds. 
                      Negative values are interpreted as infinite duration.
)doc");
  m.def("cleanup_ui", &EnkiApplication::cleanup, R"doc( 
Cleanup the Qt runtime.
)doc");
}

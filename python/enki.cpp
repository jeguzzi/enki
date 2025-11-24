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

#include "../enki/Geometry.h"
#include "../enki/PhysicalEngine.h"
#include "../enki/Types.h"
#include "../enki/robots/e-puck/EPuck.h"
#include "../enki/robots/marxbot/Marxbot.h"
#include "../enki/robots/thymio2/Thymio2.h"
#include "./enki.h"

using namespace Enki;
namespace py = pybind11;

#if CONVERT_COLOR
namespace pybind11 {
namespace detail {

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
} // namespace detail
} // namespace pybind11
#endif

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

// wrappers for robots

#define OVERRIDE_PO(cname, dname)                                              \
public:                                                                        \
  void controlStep(double dt) override {                                       \
    cname::controlStep(dt);                                                    \
    PYBIND11_OVERRIDE_IMPL(PYBIND11_TYPE(void), PYBIND11_TYPE(dname),          \
                           "control_step", dt);                                \
  }                                                                            \
                                                                               \
  void mousePressEvent(unsigned button, double pointX, double pointY,          \
                       double pointZ) override {                               \
    bool state = true;                                                         \
    cname::mousePressEvent(button, pointX, pointY, pointZ);                    \
    PYBIND11_OVERRIDE_IMPL(PYBIND11_TYPE(void), PYBIND11_TYPE(dname),          \
                           "on_touch", state, button, pointX, pointY, pointZ); \
  }                                                                            \
                                                                               \
  void mouseReleaseEvent(unsigned button) override {                           \
    double pointX, pointY, pointZ = 0;                                         \
    bool state = false;                                                        \
    cname::mouseReleaseEvent(button);                                          \
    PYBIND11_OVERRIDE_IMPL(PYBIND11_TYPE(void), PYBIND11_TYPE(dname),          \
                           "on_touch", state, button, pointX, pointY, pointZ); \
  }

struct PyPhysicalObject : public PhysicalObject,
                          public py::trampoline_self_life_support {
  using PhysicalObject::PhysicalObject;
  OVERRIDE_PO(PhysicalObject, PyPhysicalObject)
};

struct PyMarxbot : public Marxbot, public py::trampoline_self_life_support {
  using Marxbot::Marxbot;
  OVERRIDE_PO(Marxbot, PyMarxbot)
};

struct PyEPuck : public EPuck, public py::trampoline_self_life_support {
  using EPuck::EPuck;
  OVERRIDE_PO(EPuck, PyEPuck)
};

struct PyThymio2 : public Thymio2, public py::trampoline_self_life_support {
  using Thymio2::Thymio2;

  py::array_t<double> get_led_color_array() const {
    const double *data = static_cast<const double *>(ledColor[0].components);
    const std::array<ssize_t, 2> shape{static_cast<ssize_t>(Thymio2::LED_COUNT),
                                       4};
    py::array_t<double> ts(shape, data);
    py::buffer_info buf = ts.request();
    py::detail::array_proxy(ts.ptr())->flags &=
        ~py::detail::npy_api::NPY_ARRAY_WRITEABLE_;
    return ts;
  }

  void hasTouchedButton(Button button) override {
    Thymio2::hasTouchedButton(button);
    PYBIND11_OVERRIDE_IMPL(PYBIND11_TYPE(void), PYBIND11_TYPE(PyThymio2),
                           "on_button_touch", button);
  }

  OVERRIDE_PO(Thymio2, PyThymio2)
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

std::array<double, 3> get_thymio_rgb_led(const Thymio2 &thymio,
                                         Thymio2::LedIndex index) {
  const auto color = thymio.getColorLed(index);
  return {color.r() * color.a(), color.g() * color.a(), color.b() * color.a()};
}

void set_thymio_led(Thymio2 &thymio, Thymio2::LedIndex first_index, int number,
                    int index, double value) {
  if (index == -1) {
    for (int i = 0; i < number; ++i) {
      thymio.setLedIntensity(Thymio2::LedIndex(first_index + i), value);
    }
  } else if (index < number) {
    thymio.setLedIntensity(Thymio2::LedIndex(first_index + index), value);
  }
}

void set_thymio_leds(Thymio2 &thymio, Thymio2::LedIndex first_index, int number,
                     std::vector<double> values) {
  if (values.size() != number) {
    throw std::length_error("Requires " + std::to_string(number) + " values");
  }
  for (int i = 0; i < number; ++i) {
    thymio.setLedIntensity(Thymio2::LedIndex(first_index + i), values[i]);
  }
}

double get_thymio_led(const Thymio2 &thymio, Thymio2::LedIndex first_index,
                      unsigned index, int number) {
  if (index < number) {
    return thymio.getLedIntensity(Thymio2::LedIndex(first_index + index));
  }
  throw std::out_of_range("No LED at index " + std::to_string(index));
}

std::vector<double> get_thymio_leds(const Thymio2 &thymio,
                                    Thymio2::LedIndex first_index, int number) {
  std::vector<double> rs(number);
  for (int i = 0; i < number; ++i) {
    rs[i] = thymio.getLedIntensity(Thymio2::LedIndex(first_index + i));
  }
  return rs;
}

Polygon make_polygon(const std::vector<Vector> &ps) {
  Polygon p;
  bool orient = (ps[1] - ps[0]).cross(ps[2] - ps[1]) > 0;
  if (orient) {
    p.assign(ps.begin(), ps.end());
  } else {
    p.assign(ps.rbegin(), ps.rend());
  }
  return p;
}

PYBIND11_MAKE_OPAQUE(PhysicalObject::Hull)

PYBIND11_MODULE(pyenki, m) {

#if !(CONVERT_COLOR)

  py::classh<Color>(m, "Color", py::buffer_protocol(), R"doc(
Args:
    r (float): Red channel, in [0, 1], optional (default 0.0)
    g (float): Green channel, in [0, 1], optional (default 0.0)
    b (float): Blue channel, in [0, 1], optional (default 0.0)
    a (float): Alpha channel, in [0, 1], optional (default 1.0)

An RGBA color with values between 0.0 and 1.0.

Attributes:
    black (Color): readonly
    gray (Color) : readonly
    white (Color) : readonly
    red (Color) : readonly
    green (Color) : readonly
    blue (Color) : readonly
    lightgray (Color) : readonly
    darkgray (Color) : readonly
    lightred (Color) : readonly
    darkred (Color) : readonly
    lightgreen (Color) : readonly
    darkgreen (Color) : readonly
    lightblue (Color) : readonly
    darkblue (Color) : readonly
    lightyellow (Color) : readonly
    yellow (Color) : readonly
    darkyellow (Color) : readonly
    orange (Color) : readonly
    violet (Color) : readonly
    purple (Color) : readonly
    pink (Color) : readonly
    cyan (Color) : readonly

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
      .def_buffer([](Color &c) -> py::buffer_info {
        return py::buffer_info(c.components, sizeof(double),
                               py::format_descriptor<double>::format(), 1, {4},
                               {sizeof(double)});
      })
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
      .def_property_readonly_static(
          "lightgray", [](py::object /* self */) { return Color::lightGray; })
      .def_property_readonly_static(
          "darkgray", [](py::object /* self */) { return Color::darkGray; })
      .def_property_readonly_static(
          "lightred", [](py::object /* self */) { return Color::lightRed; })
      .def_property_readonly_static(
          "darkred", [](py::object /* self */) { return Color::darkRed; })
      .def_property_readonly_static(
          "lightgreen", [](py::object /* self */) { return Color::lightGreen; })
      .def_property_readonly_static(
          "darkgreen", [](py::object /* self */) { return Color::darkGreen; })
      .def_property_readonly_static(
          "lightblue", [](py::object /* self */) { return Color::lightBlue; })
      .def_property_readonly_static(
          "darkblue", [](py::object /* self */) { return Color::darkBlue; })
      .def_property_readonly_static(
          "lightyellow",
          [](py::object /* self */) { return Color::lightYellow; })
      .def_property_readonly_static(
          "yellow", [](py::object /* self */) { return Color::yellow; })
      .def_property_readonly_static(
          "darkyellow", [](py::object /* self */) { return Color::darkYellow; })
      .def_property_readonly_static(
          "orange", [](py::object /* self */) { return Color::orange; })
      .def_property_readonly_static(
          "violet", [](py::object /* self */) { return Color::violet; })
      .def_property_readonly_static(
          "purple", [](py::object /* self */) { return Color::purple; })
      .def_property_readonly_static(
          "pink", [](py::object /* self */) { return Color::pink; })
      .def_property_readonly_static(
          "cyan", [](py::object /* self */) { return Color::cyan; })
      .def_property("r", &Color::r, &Color::setR)
      .def_property("g", &Color::g, &Color::setG)
      .def_property("b", &Color::b, &Color::setB)
      .def_property("a", &Color::a, &Color::setA)
      .def_property("components", getColorComponents, setColorComponents);

#endif

  // py::bind_vector<Texture>(m, "Texture");
  // py::bind_vector<Textures>(m, "Textures");
  py::bind_vector<PhysicalObject::Hull>(m, "Hull");

  // Physical objects

  py::classh<PhysicalObject, PyPhysicalObject> po(m, "PhysicalObject", R"doc(
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
    parts (list[PhysicalObject.Part]): the parts the object is composed of.
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
    touch_callback (Callable[[PhysicalObject], None] | None): An optional function called when touch events happen.
)doc");

  py::classh<PhysicalObject::Part>(po, "Part", py::dynamic_attr(), R"doc(
Right prism that can be composed to define the geometry of a :py:class:`pyenki.PhysicalObject`.

Attributes:
    shape (Sequence[Vector]): The convex 2D polygon (positively oriented) at the base of the prism [cm].
    height (float): The height [cm].
    textures (Sequence[Sequence[Color]]): A list of textures: each texture is a list of colors for one vertical face of the prism. 
                                          Must be either empty or have contains at least one color for each face.
)doc")
      .def(py::init<double, double, double>(), py::arg("lx"), py::arg("ly"),
           py::arg("height"), R"doc(
Creates a part with a rectangular base.

Arguments:
  lx (float): The x dimension [cm]
  ly (float): The y dimension [cm]
  height (float): The height [cm]
)doc")
      .def(py::init([](const std::vector<Vector> &shape, double height,
                       const Textures &textures) {
             if (textures.size()) {
               return std::make_unique<PhysicalObject::Part>(
                   make_polygon(shape), height, textures);
             } else {
               return std::make_unique<PhysicalObject::Part>(
                   make_polygon(shape), height);
             }
           }),
           py::arg("shape"), py::arg("height"),
           py::arg("textures") = Textures{}, R"doc(
Creates a part with a polygonal base.

Arguments:
  shape (Sequence[Vector]): The convex 2D polygon (positively oriented) at the base of the prism [cm].
  height (float): The height [cm]
  textures (Sequence[Sequence[Color]]): A sequence of textures: each texture is a sequence of colors for one vertical face of the prism. 
                                        Must be either empty or have contains at least one color for each face.
)doc")
      .def_property(
          "shape",
          [](const PhysicalObject::Part &part) {
            const std::vector<Vector> &shape = part.getShape();
            return shape;
          },
          nullptr)
      .def_property("height", &PhysicalObject::Part::getHeight, nullptr)
      .def_property("textures", &PhysicalObject::Part::getTextures, nullptr)
      .def("__hash__",
           [](const PhysicalObject::Part &part) {
             const std::vector<Vector> &shape = part.getShape();
             py::module_ np = py::module_::import("numpy");
             const auto shape_bytes =
                 np.attr("asarray")(py::cast(shape)).attr("tobytes")();
             const auto ts = part.getTextures();
             if (ts.size()) {
               const auto textures_bytes =
                   np.attr("concat")(py::cast(ts)).attr("tobytes")();
               const auto t = py::make_tuple(part.getHeight(), shape_bytes,
                                             textures_bytes);
               return py::hash(t);

             } else {
               const auto t = py::make_tuple(part.getHeight(), shape_bytes);
               return py::hash(t);
             }
           })
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("contains", &PhysicalObject::Part::contains, py::arg("point"),
           py::arg("tolerance") = 0);

  po.def(py::init([](const std::vector<PhysicalObject::Part> &parts,
                     double mass, const Color &color) {
           auto obj = std::make_shared<PhysicalObject>();
           PhysicalObject::Hull hull;
           hull.assign(parts.begin(), parts.end());
           obj->setCustomHull(hull, mass);
           obj->setColor(color);
           return obj;
         }),
         py::arg("parts"), py::arg("mass"), py::arg("color") = Color::black,
         R"doc(
Creates an object composed of parts.

Arguments:
  parts (PhysicalObject.Part): The parts.
  mass (float): The mass in kg.
  color (Color): The color.
)doc")
      .def(py::init([](double radius, double height, double mass,
                       const Color &color = Color()) {
             auto obj = std::make_shared<PyPhysicalObject>();
             obj->setCylindric(radius, height, mass);
             obj->setColor(color);
             return obj;
           }),
           py::arg("radius"), py::arg("height"), py::arg("mass"),
           py::arg("color") = Color(), R"doc(
Creates a cylinder.

Arguments:
  radius (float): The radius in cm.
  height (float): The height in cm.
  mass (float): The mass in kg.
  color (Color): The color.
)doc")
      .def(py::init([](double l1, double l2, double height, double mass,
                       const Color &color = Color()) {
             auto obj = std::make_shared<PhysicalObject>();
             obj->setRectangular(l1, l2, height, mass);
             obj->setColor(color);
             return obj;
           }),
           py::arg("lx"), py::arg("ly"), py::arg("height"), py::arg("mass"),
           py::arg("color") = Color(), R"doc(
Creates a rectangular prism.

Arguments:
  lx (float): the side length in cm (x).
  ly (float): the side length in cm (y).
  height (float): The height in cm.
  mass (float): The mass in kg.
  color (Color): The color.
)doc")
      .def(py::init([](const std::vector<Vector> &shape, double height,
                       double mass, const Color &color = Color(),
                       const Textures &textures = {}) {
             auto c = std::make_shared<PhysicalObject>();
             if (textures.size()) {
               PhysicalObject::Part part(make_polygon(shape), height, textures);
               c->setCustomHull(PhysicalObject::Hull(part), mass);
             } else {
               c->setCustomHull(PhysicalObject::Hull(PhysicalObject::Part(
                                    make_polygon(shape), height)),
                                mass);
             }
             c->setColor(color);
             return c;
           }),
           py::arg("shape"), py::arg("height"), py::arg("mass"),
           py::arg("color") = Color(), py::arg("textures") = Textures{},
           R"doc(
Creates an vertical prism with a convex polygonal base.

Arguments:
  shape (Sequence[Vector]): The vertices polygonal base in cm. Must be convex.
  height (float): The height in cm.
  mass (float): The mass in kg.
  color (Color): The color.
  textures (Sequence[Color]): if not empty, defines the colors of each face.
)doc")
      .def_readonly("uid", &PhysicalObject::uid)
      // .def_property(
      //     "parts",
      //     // &PhysicalObject::getHull,
      //     [](const PhysicalObject &obj) {
      //       const std::vector<PhysicalObject::Part> &ps = obj.getHull();
      //       return ps;
      //     },
      //     nullptr)
      .def_property("parts", &PhysicalObject::getHull, nullptr)
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
      .def_property("position", &PhysicalObject::getPosition,
                    &PhysicalObject::setPosition)
      .def_property("angle", &PhysicalObject::getAngle,
                    &PhysicalObject::setAngle)
      .def_readwrite("velocity", &PhysicalObject::speed)
      .def_readwrite("angular_speed", &PhysicalObject::angSpeed)
      .def_property("collision_callback", &PhysicalObject::getCollisionCallback,
                    &PhysicalObject::setCollisionCallback,
                    py::keep_alive<1, 2>())
      .def_property("control_step_callback",
                    &PhysicalObject::getControlCallback,
                    &PhysicalObject::setControlCallback, py::keep_alive<1, 2>())
      .def_property("touch_callback", &PhysicalObject::getTouchCallback,
                    &PhysicalObject::setTouchCallback, py::keep_alive<1, 2>())
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
      .def(
          "on_touch",
          [](PyPhysicalObject &o, bool state, unsigned button, double x,
             double y, double z) { o.touchEvent(state, button, x, y, z); },
          py::arg("state"), py::arg("button"), py::arg("x"), py::arg("y"),
          py::arg("z"), R"doc(
Called after a touch event.

Can be overridden by sub-classes to react to touch events. 
Alternatively, users can assign a callback as :py:attr:`touch_callback`.

Arguments:
  state (bool): True for press, False for release.
  button (int): mouse button index: 0 (left), 1 (right) or 2 (middle).
  x (float): cursor x-coordinate (cm).
  y (float): cursor y-coordinate (cm).
  z (float): cursor z-coordinate (cm). 
)doc")
      .def(
          "touch",
          [](PyPhysicalObject &o, bool state, unsigned button, double x,
             double y, double z) { o.touchEvent(state, button, x, y, z); },
          py::arg("state"), py::arg("button"), py::arg("x"), py::arg("y"),
          py::arg("z"), R"doc(
Trigger a touch event.

Arguments:
  state (bool): True for press, False for release.
  button (int): mouse button index: 0 (left), 1 (right) or 2 (middle).
  x (float): cursor x-coordinate (cm).
  y (float): cursor y-coordinate (cm).
  z (float): cursor z-coordinate (cm). 
)doc")
      // TODO(OLD): warning setting the "color" property at run time using the
      // viewer from the non-gui thread will lead to a crash because it will do
      // an OpenGL call from that thread
      .def_property("color", &PhysicalObject::getColor,
                    &PhysicalObject::setColor)
      .def("contains", &PhysicalObject::contains, py::arg("point"),
           py::arg("tolerance") = 0);

  // Robots

  py::classh<Robot, PhysicalObject>(m, "Robot", "Base class for all robots");

  py::classh<DifferentialWheeled, Robot, PhysicalObject>(
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

  py::classh<Marxbot, PyMarxbot, DifferentialWheeled, PhysicalObject>(
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
            scanner_distances (Array1D): An array of 180 radial distances,
                ordered from -180 degrees to 180 degrees, in centimeters (readonly).
            scanner_image (Array2D): An rgba array between 0 and 1 of shape ``(180, 4)`` (readonly).
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

  py::classh<EPuck, PyEPuck, DifferentialWheeled, PhysicalObject>(m, "EPuck",
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
    >>> obj = pyenki.PhysicalObject(radius=2.0, height=5.0, mass=-1, 
                                    pyenki.Color(0.3, 0.7, 0))
    >>> world.add_object(obj)
    >>> world.step(0.1)
    >>> epuck.prox_values
    array([98.52232283,  3.30197324, ...
    >>> epuck.camera_image
    array([[0.5, 0.5, 0.5, 1. ] ...

Attributes:

    prox_values (Array1D): An array of 8 proximity sensor readings, one for each sensors (readonly).
    prox_distances (Array1D): An array of 8 distances between proximity sensor and nearest obstacles, one for each sensors (readonly).
        please note that this value would *not* directly be accessible by a real robot (readonly).
    scan (Array1D): An array of 64 radial distances, ordered from -180 degrees to 180 degrees, in centimeters.
    camera_image (Array2D): An rgba array between 0 and 1 of shape ``(60, 4)`` (readonly).
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

  py::classh<Thymio2, PyThymio2, DifferentialWheeled, PhysicalObject> thymio(
      m, "Thymio2", R"doc( 
A :py:class:`DifferentialWheeled` Thymio2 robot.

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

Attributes:
    prox_values (Array1D): An array of 7 proximity sensor readings, one for each sensors (readonly).
        The first 5 entries are from frontal sensors ordered from left to right.
        The last two entries are from rear sensors  ordered from left to right.
    prox_distances (Array1D): A list of 7 distances between proximity sensor and nearest obstancle, one for each sensors;
        please note that this value would *not* directly be accessible by a real robot (readonly).
        The first 5 entries are from frontal sensors ordered from left to right.
        The last two entries are from rear sensors  ordered from left to right.
    prox_comm_tx (int): The integer payload to be sent. The real robot can only send 11 bits,
        therefore to be compliant we should limit the value between 0 and 2047.
    prox_comm_enabled (bool): Enable/disable proximity communication.
    prox_comm_events (list[IRCommEvent]): A list of events, one for every received message during the last control step (readonly).
    ground_values (Array1D): An array of 2 ground sensor readings, one for each sensors (readonly)
    button_touch_callback (Callable[[Thymio2, int], None] | None): An optional function called when button touch events happen.
    leds_buttons: list[float]: The intensities of the 4 buttons LEDs.
    leds_circle: list[float]: The intensities of the 8 circle LEDs.
    leds_prox: list[float]: The intensities of the 8 proximity LEDs.
    led_left_red: float: The intensity of the left red LED.
    led_left_blue: float: The intensity of the left blue LED.
    led_right_red: float: The intensity of the right red LED.
    led_right_blue: float: The intensity of the right blue LED.
    buttons: list[bool]: Whether the buttons are being pressed.
)doc");

  py::classh<IRCommEvent>(thymio, "IRCommEvent", R"doc( 
This event is created each time a message is received by at least one proximity sensor.
The sensors that do not receive the message, have the corresponding payloads and intensities set to zero.

Attributes:
    rx_value (int): The received message payload (readonly)
    payloads (IntArray1D): An array of 7 integer payloads, one for each sensors (readonly).
        The first 5 entries are from frontal sensors ordered from left to right.
        The last two entries are from rear sensors  ordered from left to right.
    intensities (IntArray1D): An array of 7 integer intensities, one for each sensors (readonly).
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

  py::native_enum<Thymio2::Button>(thymio, "Button", "enum.Enum", R"doc(
Identify one of the five touch button of the Thymio2.
)doc")
      .value("CENTER", Thymio2::Button::CENTER, R"doc(
)doc")
      .value("FORWARD", Thymio2::Button::FORWARD, R"doc(
)doc")
      .value("BACKWARD", Thymio2::Button::BACKWARD, R"doc(
)doc")
      .value("LEFT", Thymio2::Button::LEFT, R"doc(
)doc")
      .value("RIGHT", Thymio2::Button::RIGHT, R"doc(
)doc")
      .finalize();

  thymio.def(py::init<>(), "Constructs an instance")
      .def_property("button_touch_callback", &Thymio2::getButtonTouchCallback,
                    &Thymio2::setButtonTouchCallback, py::keep_alive<1, 2>())
      .def(
          "on_button_touch",
          [](PyThymio2 &t, Thymio2::Button button) {
            t.hasTouchedButton(button);
          },
          py::arg("index"), R"doc(
Called after a button is touched.

Can be overridden by sub-classes to react to button touch events. 
Alternatively, users can assign a callback as :py:attr:`button_touch_callback`.

Arguments:
  index (int): The button being touched.
)doc")
      .def("touch_button", &Thymio2::touchButton, py::arg("button"), R"doc( 
Touches one of the buttons on top of the robot.

Args:
    index (int): the index of the button
)doc")
      .def_property(
          "buttons",
          [](const Thymio2 &r) {
            std::vector<bool> vs(5);
            vs.assign(r.getButtonValues(), r.getButtonValues() + 5);
            return vs;
          },
          nullptr)
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
          "ground_values",
          [](const Thymio2 &r) {
            const std::vector<double> vs{r.groundSensor0.getValue(),
                                         r.groundSensor1.getValue()};
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
      .def_property(
          "led_colors",
          [](const PyThymio2 &t) { return t.get_led_color_array(); }, nullptr)
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
          "get_led_top",
          [](const Thymio2 &r) {
            return get_thymio_rgb_led(r, Thymio2::LedIndex::TOP);
          },
          R"doc( 
Reads the top RGB LED color

Returns:
    list[float]: the value of the ``[red, blue, green]`` channels
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
          "get_led_top",
          [](const Thymio2 &r) {
            return get_thymio_rgb_led(r, Thymio2::LedIndex::BOTTOM_LEFT);
          },
          R"doc( 
Reads the bottom left RGB LED color

Returns:
    list[float]: the value of the ``[red, blue, green]`` channels
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
          "get_led_bottom_right",
          [](const Thymio2 &r) {
            return get_thymio_rgb_led(r, Thymio2::LedIndex::BOTTOM_RIGHT);
          },
          R"doc( 
Reads the bottom left RGB LED color

Returns:
    list[float]: the value of the ``[red, blue, green]`` channels
)doc")
      .def(
          "set_led_buttons",
          [](Thymio2 &r, int index, double value) {
            set_thymio_led(r, Thymio2::LedIndex::BUTTON_UP, 4, index, value);
          },
          py::arg("index"), py::arg("value"), R"doc(
Control the four button LEDs

Args:
    index (int): the index of the LED. Set to -1 to control all LEDs.
    value (float): the desired intensity between 0 and 1.
)doc")
      .def(
          "get_led_buttons",
          [](const Thymio2 &r, unsigned index) {
            return get_thymio_led(r, Thymio2::LedIndex::BUTTON_UP, 4, index);
          },
          py::arg("index"), R"doc(
Reads one of four button LEDs

Args:
    index (int): the index of the LED (between 0 and 3)
Returns:
    float: the intensity between 0 and 1.
)doc")
      .def_property(
          "leds_buttons",
          [](const Thymio2 &r) {
            get_thymio_leds(r, Thymio2::LedIndex::BUTTON_UP, 4);
          },
          [](Thymio2 &r, std::vector<double> values) {
            set_thymio_leds(r, Thymio2::LedIndex::BUTTON_UP, 4, values);
          })
      .def(
          "set_led_circle",
          [](Thymio2 &r, int index, double value) {
            set_thymio_led(r, Thymio2::LedIndex::RING_0, 8, index, value);
          },
          py::arg("index"), py::arg("value"), R"doc(
Control the 8 circle LEDs

Args:
    index (int): the index of the LED. Set to -1 to control all LEDs.
    value (float): the desired intensity between 0 and 1.
)doc")
      .def(
          "get_led_circle",
          [](const Thymio2 &r, unsigned index) {
            return get_thymio_led(r, Thymio2::LedIndex::RING_0, 8, index);
          },
          py::arg("index"), R"doc(
Reads one of 8 circle LEDs

Args:
    index (int): the index of the LED (between 0 and 7)
Returns:
    float: the intensity between 0 and 1.
)doc")
      .def_property(
          "leds_circle",
          [](const Thymio2 &r) {
            get_thymio_leds(r, Thymio2::LedIndex::RING_0, 8);
          },
          [](Thymio2 &r, std::vector<double> values) {
            set_thymio_leds(r, Thymio2::LedIndex::RING_0, 8, values);
          })
      .def(
          "set_led_prox",
          [](Thymio2 &r, int index, double value) {
            set_thymio_led(r, Thymio2::LedIndex::IR_FRONT_0, 8, index, value);
          },
          py::arg("index"), py::arg("value"), R"doc(
Control the 8 proximity LEDs

Args:
    index (int): the index of the LED. Set to -1 to control all LEDs.
    value (float): the desired intensity between 0 and 1.
)doc")
      .def(
          "get_led_prox",
          [](const Thymio2 &r, unsigned index) {
            return get_thymio_led(r, Thymio2::LedIndex::IR_FRONT_0, 8, index);
          },
          py::arg("index"), R"doc(
Reads one of 8 proximity LEDs

Args:
    index (int): the index of the LED (between 0 and 7)
Returns:
    float: the intensity between 0 and 1.
)doc")
      .def_property(
          "leds_prox",
          [](const Thymio2 &r) {
            get_thymio_leds(r, Thymio2::LedIndex::IR_FRONT_0, 8);
          },
          [](Thymio2 &r, std::vector<double> values) {
            set_thymio_leds(r, Thymio2::LedIndex::IR_FRONT_0, 8, values);
          })
      .def_property(
          "led_left_red",
          [](const Thymio2 &r) {
            r.getLedIntensity(Thymio2::LedIndex::LEFT_RED);
          },
          [](Thymio2 &r, double value) {
            r.setLedIntensity(Thymio2::LedIndex::LEFT_RED, value);
          })
      .def_property(
          "led_left_blue",
          [](const Thymio2 &r) {
            r.getLedIntensity(Thymio2::LedIndex::LEFT_BLUE);
          },
          [](Thymio2 &r, double value) {
            r.setLedIntensity(Thymio2::LedIndex::LEFT_BLUE, value);
          })
      .def_property(
          "led_right_red",
          [](const Thymio2 &r) {
            r.getLedIntensity(Thymio2::LedIndex::RIGHT_RED);
          },
          [](Thymio2 &r, double value) {
            r.setLedIntensity(Thymio2::LedIndex::RIGHT_RED, value);
          })
      .def_property(
          "led_right_blue",
          [](const Thymio2 &r) {
            r.getLedIntensity(Thymio2::LedIndex::RIGHT_BLUE);
          },
          [](Thymio2 &r, double value) {
            r.setLedIntensity(Thymio2::LedIndex::RIGHT_BLUE, value);
          });

  py::classh<PyWorld> world(m, "World", R"doc(
The world is the container of all objects and robots.
It is either

- a rectangular arena with walls at the borders::
    
    World(lx = ..., ly = ...)

- a circular area with walls at the borders::

    World(radius = ...)

- or an infinite surface with no walls::

    World()

Args:
    lx (float): The rectangular world x-size in centimeters
    ly (float): The rectangular world y-size in centimeters
    radius (float): The circular world radius in centimeters
    walls_color (Color): Optional wall color, default is ``Color.gray``
    ground_texture (World.GroundTexture | None): Optional ground texture, default is an empty image.
    seed (int): The random seed

Example::

    import pyenki

    world = pyenki.World()
    thymio = Thymio2()
    world.add_object(thymio)
    wall = pyenki.PhysicalObject(lx=10, ly=50, height=5, mass=1,
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
    random_seed (numpy.random.Generator): The random seed
    lx (float): the world x-size [cm]
    ly (float): the world y-size [cm]
    radius(float) : the world radius [cm]
    walls_type (World.WallsType): the type of boundary walls.
    ground_texture (World.GroundTexture): an optional image to color the ground (readonly). 
)doc");

  py::classh<World::GroundTexture>(world, "GroundTexture",
                                   py::buffer_protocol(), R"doc(
2-D Texture for ground stored as a ARGB array (0xAARRGGBB in little endian).
Users should access class data using the buffer protocol, i.e.

>>> image = np.zeros((200, 100, 4), dtype=np.uint8)
>>> gt = World.GroundTexture(image)
>>> gt.width, gt.height
(100, 200)
>>> data = numpy.asarray(gt)
>>> data.shape, data[0, 0, 0]
((100, 200, 4), np.uint8(0))

Attributes:
  
    width (int): the width
    height (int): the height
)doc")
      .def(py::init<>(), "Creates an empty ground texture")
      .def(py::init([](const py::array_t<uint8_t, py::array::c_style |
                                                      py::array::forcecast>
                           image) {
             py::buffer_info buf = image.request();
             if (buf.ndim != 3 || buf.shape[2] != 4) {
               throw std::runtime_error(
                   "Buffer must have shape (height, width, 4).");
             }
             std::vector<uint32_t> data(buf.shape[1] * buf.shape[0]);
             std::copy_n(reinterpret_cast<uint8_t *>(buf.ptr),
                         buf.shape[2] * buf.shape[1] * buf.shape[0],
                         reinterpret_cast<uint8_t *>(&data[0]));

             return std::make_unique<World::GroundTexture>(
                 buf.shape[1], buf.shape[0], data.data());
           }),
           py::arg("data"), R"doc(
Creates an ground texture with a copy of the ARGB data

Args:
  data (ARGBImage): A numpy array of shape ``(height, width, 4)``
                        and type :py:attr:`numpy.uint8` storing ARGB pixels.
)doc")
      .def_buffer([](World::GroundTexture &c) -> py::buffer_info {
        const std::array<ssize_t, 3> shape{c.height, c.width, 4};
        const std::array<ssize_t, 3> strides{
            static_cast<ssize_t>(c.width * 4 * sizeof(uint8_t)),
            4 * sizeof(uint8_t), sizeof(uint8_t)};
        // READONLY (for now)
        return py::buffer_info(c.data.data(), sizeof(uint8_t),
                               py::format_descriptor<uint8_t>::format(), 3,
                               shape, strides, true);
      })
      .def_readonly("width", &World::GroundTexture::width)
      .def_readonly("height", &World::GroundTexture::height);

  py::native_enum<World::WallsType>(world, "WallsType", "enum.Enum", R"doc(
Describes the type of boundary walls.
)doc")
      .value("SQUARE", World::WallsType::WALLS_SQUARE, R"doc(
A rectangular boundary wall of size (:py:attr:`World.width`, :py:attr:`World.height`).
)doc")
      .value("CIRCULAR", World::WallsType::WALLS_CIRCULAR, R"doc(
A circular boundary wall of radius :py:attr:`World.radius`.
)doc")
      .value("NONE", World::WallsType::WALLS_NONE, R"doc(
No boundary walls.
)doc")
      .finalize();

  world.def(py::init<unsigned long>(), py::arg("seed") = 0)
      .def(py::init<double, double, const Color &,
                    const std::optional<World::GroundTexture> &,
                    unsigned long>(),
           py::arg("lx"), py::arg("ly"),
           py::arg("walls_color") = Color::gray,
           py::arg("ground_texture") = std::nullopt, py::arg("seed") = 0)
      .def(py::init<double, const Color &,
                    const std::optional<World::GroundTexture> &,
                    unsigned long>(),
           py::arg("radius"), py::arg("walls_color") = Color::gray,
           py::arg("ground_texture") = std::nullopt, py::arg("seed") = 0)
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
      .def("copy_random_generator", &PyWorld::copyRandom, py::arg("world"),
           R"doc( 
Copy the random generator from another world

Args:
    world (World): the other world.
)doc")
      .def_readonly("radius", &PyWorld::r)
      .def_readonly("lx", &PyWorld::w)
      .def_readonly("ly", &PyWorld::h)
      .def_readonly("walls_color", &PyWorld::color)
      .def_readonly("walls_type", &PyWorld::wallsType)
      .def_readonly("ground_texture", &PyWorld::groundTexture)
      .def_property("has_ground_texture", &PyWorld::hasGroundTexture, nullptr)
      .def("get_ground_color", &PyWorld::getGroundColor, py::arg("position"),
           R"doc( 
Returns the color of the floor at a given position

Args:
    position (Vector): the position.

Returns:
    Color: the color at the position.
)doc")
      .def_property("random_seed", &PyWorld::getRandomSeed,
                    &PyWorld::setRandomSeed)
      .def_property("random_generator", &PyWorld::getRandom,
                    &PyWorld::setRandom)
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
)doc");
}

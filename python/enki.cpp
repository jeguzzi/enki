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
*/

#include <Python.h>

#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>

#include "../enki/Types.h"
#include "../enki/Geometry.h"
#include "../enki/PhysicalEngine.h"
#include "../enki/robots/e-puck/EPuck.h"
#include "../enki/robots/thymio2/Thymio2.h"
#include "../viewer/Viewer.h"
#include <QApplication>
#include <QImage>
#include <QGLWidget>

using namespace Enki;
namespace py = pybind11;

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

using namespace pybind11;

tuple getColorComponents(const Color& color)
{
	return make_tuple(
		color.components[0],
		color.components[1],
		color.components[2],
		color.components[3]
	);
}

void setColorComponents(Color& color, tuple values)
{
	if (len(values) != 4)
		throw std::runtime_error("Tuple used to set components must be of length 4");
	color.components[0] = values[0].cast<double>();
	color.components[1] = values[1].cast<double>();
	color.components[2] = values[2].cast<double>();
	color.components[3] = values[3].cast<double>();
}

static World::GroundTexture loadTexture(const std::string& fileName)
{
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
				throw std::runtime_error("Early end-of-file: " + ppmFileName);
			r = (r * 255) / valuesScale;
			g = (g * 255) / valuesScale;
			b = (b * 255) / valuesScale;
			t.data.push_back(r|(g<<8)|(b<<16));
		}
	}
	
	return t;*/
	QImage image(fileName.c_str());
	QImage gt(image.convertToFormat(QImage::Format_ARGB32));
	
	#if QT_VERSION >= QT_VERSION_CHECK(4,7,0)
	return World::GroundTexture(gt.width(), gt.height(), (const uint32_t*)gt.constBits());
	#else
	return World::GroundTexture(gt.width(), gt.height(), (uint32_t*)gt.bits());
	#endif
}

// wrappers for objects

struct CircularPhysicalObject: public PhysicalObject
{
	CircularPhysicalObject(double radius, double height, double mass, const Color& color = Color())
	{
		setCylindric(radius, height, mass);
		setColor(color);
	}
};

struct RectangularPhysicalObject: public PhysicalObject
{
	RectangularPhysicalObject(double l1, double l2, double height, double mass, const Color& color = Color())
	{
		setRectangular(l1, l2, height, mass);
		setColor(color);
	}
};

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


struct EPuckWrap: EPuck
{
	EPuckWrap():
		EPuck(CAPABILITY_BASIC_SENSORS|CAPABILITY_CAMERA)
	{}
	
	list getProxSensorValues(void)
	{
		list l;
		l.append(infraredSensor0.getValue());
		l.append(infraredSensor1.getValue());
		l.append(infraredSensor2.getValue());
		l.append(infraredSensor3.getValue());
		l.append(infraredSensor4.getValue());
		l.append(infraredSensor5.getValue());
		l.append(infraredSensor6.getValue());
		l.append(infraredSensor7.getValue());
		return l;
	}
	
	list getProxSensorDistances(void)
	{
		list l;
		l.append(infraredSensor0.getDist());
		l.append(infraredSensor1.getDist());
		l.append(infraredSensor2.getDist());
		l.append(infraredSensor3.getDist());
		l.append(infraredSensor4.getDist());
		l.append(infraredSensor5.getDist());
		l.append(infraredSensor6.getDist());
		l.append(infraredSensor7.getDist());
		return l;
	}
	
	Texture getCameraImage(void)
	{
		Texture texture;
		texture.reserve(camera.image.size());
		for (size_t i = 0; i < camera.image.size(); ++i)
			texture.push_back(camera.image[i]);
		return texture;
	}

	OVERRIDE_CONTROL_STEP(EPuck, EPuckWrap)
};

struct Thymio2Wrap: Thymio2
{

	list getProxSensorValues(void)
	{
		list l;
		l.append(infraredSensor0.getValue());
		l.append(infraredSensor1.getValue());
		l.append(infraredSensor2.getValue());
		l.append(infraredSensor3.getValue());
		l.append(infraredSensor4.getValue());
		l.append(infraredSensor5.getValue());
		l.append(infraredSensor6.getValue());
		return l;
	}
	
	list getProxSensorDistances(void)
	{
		list l;
		l.append(infraredSensor0.getDist());
		l.append(infraredSensor1.getDist());
		l.append(infraredSensor2.getDist());
		l.append(infraredSensor3.getDist());
		l.append(infraredSensor4.getDist());
		l.append(infraredSensor5.getDist());
		l.append(infraredSensor6.getDist());
		return l;
	}
	
	list getGroundSensorValues(void)
	{
		list l;
		l.append(groundSensor0.getValue());
		l.append(groundSensor1.getValue());
		return l;
	}

	void setLedIntensity(int index, double intensity) {
		Thymio2::setLedIntensity((LedIndex)index, intensity);
	}

	void setLedColor(int index, const Color& color) {
		Thymio2::setLedColor((LedIndex)index, color);
	}

	OVERRIDE_CONTROL_STEP(Thymio2, Thymio2Wrap)
};

struct PythonViewer: public ViewerWidget
{
	PyThreadState *pythonSavedState;
	 
	PythonViewer(World& world, Vector camPos, double camAltitude, double camYaw, double camPitch, double _wallsHeight):
		ViewerWidget(&world),
		pythonSavedState(0)
	{
		camera.pos.setX(camPos.x);
		camera.pos.setY(camPos.y);
		camera.altitude = camAltitude;
		camera.yaw = camYaw;
		camera.pitch = camPitch;
		wallsHeight = _wallsHeight;
		
		managedObjectsAliases[&typeid(EPuckWrap)] = &typeid(EPuck);
		managedObjectsAliases[&typeid(Thymio2Wrap)] = &typeid(Thymio2);
	}
	
	void timerEvent(QTimerEvent * event)
	{
		// get back Python lock
		if (pythonSavedState)
			PyEval_RestoreThread(pythonSavedState);
		// touch Python objects while locked
		ViewerWidget::timerEvent(event);
		// release Python lock
		if (pythonSavedState)
			pythonSavedState = PyEval_SaveThread();
	}
};

void runInViewer(World& world, Vector camPos = Vector(0,0), double camAltitude = 0, double camYaw = 0, double camPitch = 0, double wallsHeight = 10)
{
	int argc(1);
	char* argv[1] = {(char*)"dummy"}; // FIXME: recovery sys.argv
	QApplication app(argc, argv);
	PythonViewer viewer(world, camPos, camAltitude, camYaw, camPitch, wallsHeight);
	viewer.setWindowTitle("PyEnki Viewer");
	viewer.show();
	viewer.pythonSavedState = PyEval_SaveThread();
	app.exec();
	if (viewer.pythonSavedState)
		PyEval_RestoreThread(viewer.pythonSavedState);
}

void run(World& world, unsigned steps)
{
	for (unsigned i = 0; i < steps; ++i)
		world.step(1./30., 3);
}

class WorldWithTexturedGround: public World {
	using World::World;
};

PYBIND11_MAKE_OPAQUE(Texture)
PYBIND11_MAKE_OPAQUE(Textures)

PYBIND11_MODULE(pyenki, m) {
  options options;
#if PYBIND11_VERSION_MAJOR >= 2 && PYBIND11_VERSION_MINOR >= 10
  options.disable_enum_members_docstring();
#endif
	
	// TODO: complete doc
	
#if !(CONVERT_COLOR)

	class_<Color>(m, "Color", "A color in RGBA")
		.def(init<double, double, double, double>(), py::arg("r") = 0, py::arg("g") = 0, py::arg("b") = 0, py::arg("a") = 1)
		.def(self += double())
		.def(self + double())
		.def(self -= double())
		.def(self - double())
		.def(self *= double())
		.def(self * double())
		.def(self /= double())
		.def(self / double())
		.def(self += self)
		.def(self + self)
		.def(self -= self)
		.def(self - self)
		.def(self == self)
		.def(self != self)
		// .def("__repr__", &Color::toString)
		.def("__repr__", [](const Color &color) {
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
		.def("threshold", &Color::threshold)
		.def("toGray", &Color::toGray)
		.def_property_readonly_static("black", [](py::object /* self */) { return Color::black; })
		.def_property_readonly_static("white", [](py::object /* self */) { return Color::white; })
		.def_property_readonly_static("gray", [](py::object /* self */) { return Color::gray; })
		.def_property_readonly_static("red", [](py::object /* self */) { return Color::red; })
		.def_property_readonly_static("green", [](py::object /* self */) { return Color::green; })
		.def_property_readonly_static("blue", [](py::object /* self */) { return Color::blue; })
		.def_property("r", &Color::r, &Color::setR)
		.def_property("g", &Color::g, &Color::setG)
		.def_property("b", &Color::b, &Color::setB)
		.def_property("a", &Color::a, &Color::setA)
		.def_property("components", getColorComponents, setColorComponents)
	;

#endif

	py::bind_vector<Texture>(m, "Texture");
	py::bind_vector<Textures>(m, "Textures");

	// Physical objects
	
	class_<PhysicalObject>(m, "PhysicalObject", "")
		.def_property("radius", &PhysicalObject::getRadius, nullptr)
		.def_property("height", &PhysicalObject::getHeight, nullptr)
		.def_property("isCylindric", &PhysicalObject::isCylindric, nullptr)
		.def_property("mass", &PhysicalObject::getMass, nullptr)
		.def_property("momentOfInertia", &PhysicalObject::getMomentOfInertia, nullptr)
		.def_property("interlacedDistance", &PhysicalObject::getInterlacedDistance, nullptr)
		.def_readwrite("collisionElasticity", &PhysicalObject::collisionElasticity)
		.def_readwrite("dryFrictionCoefficient", &PhysicalObject::dryFrictionCoefficient)
		.def_readwrite("viscousFrictionCoefficient", &PhysicalObject::viscousFrictionCoefficient)
		.def_readwrite("viscousMomentFrictionCoefficient", &PhysicalObject::viscousMomentFrictionCoefficient)
		.def_readwrite("pos", &PhysicalObject::pos)
		.def_readwrite("angle", &PhysicalObject::angle)
		.def_readwrite("speed", &PhysicalObject::speed)
		.def_readwrite("angSpeed", &PhysicalObject::angSpeed)
		.def_property("color",  &PhysicalObject::getColor, &PhysicalObject::setColor)
		// warning setting the "color" property at run time using the viewer from the non-gui thread will lead to a crash because it will do an OpenGL call from that thread
	;
	
  class_<CircularPhysicalObject>(m, "CircularPhysicalObject", "")
  	.def(init<double, double, double, const Color &>(), 
  		   py::arg("radius"), py::arg("height"), py::arg("mass"), py::arg("color") = Color());

  class_<RectangularPhysicalObject>(m, "RectangularPhysicalObject", "")
  	.def(init<double, double, double, double, const Color &>(), 
  		   py::arg("l1"), py::arg("l2"), py::arg("height"), py::arg("mass"), py::arg("color") = Color());

	// Robots
	
	class_<Robot, PhysicalObject>(m, "Robot", "");
	
	class_<DifferentialWheeled, Robot, PhysicalObject>(
      m, "DifferentialWheeled", "")
		.def_readwrite("leftSpeed", &DifferentialWheeled::leftSpeed)
		.def_readwrite("rightSpeed", &DifferentialWheeled::rightSpeed)
		.def_readonly("leftEncoder", &DifferentialWheeled::leftEncoder)
		.def_readonly("rightEncoder", &DifferentialWheeled::rightEncoder)
		.def_readonly("leftOdometry", &DifferentialWheeled::leftOdometry)
		.def_readonly("rightOdometry", &DifferentialWheeled::rightOdometry)
		.def("resetEncoders", &DifferentialWheeled::resetEncoders)
	;
	
	class_<EPuckWrap, DifferentialWheeled, PhysicalObject>(m, "EPuck", "")
	  .def(init<>())
		.def("controlStep", &EPuckWrap::controlStep)
		.def("setLedRing", &EPuckWrap::setLedRing)
		.def_property("proximitySensorValues", &EPuckWrap::getProxSensorValues, nullptr)
		.def_property("proximitySensorDistances", &EPuckWrap::getProxSensorDistances, nullptr)
		.def_property("cameraImage", &EPuckWrap::getCameraImage, nullptr)
	;
	
	class_<Thymio2Wrap, DifferentialWheeled, PhysicalObject>(m, "Thymio2", "")
		.def(init<>())
		.def("controlStep", &Thymio2Wrap::controlStep)
		.def("setLedIntensity", &Thymio2Wrap::setLedIntensity)
		.def("setLedColor", &Thymio2Wrap::setLedColor)
		.def_property("proximitySensorValues", &Thymio2Wrap::getProxSensorValues, nullptr)
		.def_property("proximitySensorDistances", &Thymio2Wrap::getProxSensorDistances, nullptr)
		.def_property("groundSensorValues", &Thymio2Wrap::getGroundSensorValues, nullptr)
	;
	
	// World
	
	class_<World>(m, "World",
		"The world is the container of all objects and robots.\n"
		"It is either a rectangular arena with walls at all sides, a circular area with walls, or an infinite surface."
	)
    .def(py::init([]() {
           auto w = std::make_unique<World>();
           w->takeObjectOwnership = false;
           return w;
         }))
    .def(py::init([](double width, double height,
                     const Color &wallsColor = Color::gray) {
           auto w = std::make_unique<World>(width, height, wallsColor);
           w->takeObjectOwnership = false;
           return w;
         }),
         py::arg("width"), py::arg("height"),
         py::arg("walls_color") = Color::gray)
    .def(py::init([](double radius, const Color &wallsColor = Color::gray) {
           auto w = std::make_unique<World>(radius, wallsColor);
           w->takeObjectOwnership = false;
           return w;
         }),
         py::arg("radius"), py::arg("walls_color") = Color::gray)
		.def("step", &World::step, py::arg("dt"), py::arg("physics_oversampling") = 3)
		.def("addObject", &World::addObject, py::keep_alive<1, 2>())
		.def("removeObject", &World::removeObject)
		.def("setRandomSeed", &World::setRandomSeed)
		.def("run", run)
		.def("runInViewer", [](World & world, Vector camPos = Vector(0,0), double camAltitude = 0, double camYaw = 0, double camPitch = 0, double wallsHeight = 10) {
			runInViewer(world, camPos, camAltitude, camYaw, camPitch, wallsHeight);
		}, py::arg("camPos") = Vector(0,0), py::arg("camAltitude") = 0, py::arg("camYaw") = 0, py::arg("camPitch") = 0, py::arg("wallsHeight") =10)
	;
	
	class_<WorldWithTexturedGround, World>(m, "WorldWithTexturedGround", "")
    .def(py::init([](double width, double height,
                     const std::string& ppmFileName, const Color& wallsColor = Color::gray) {
           auto w = std::make_unique<WorldWithTexturedGround>(width, height, wallsColor, loadTexture(ppmFileName));
           w->takeObjectOwnership = false;
           return w;
         }),
         py::arg("width"), py::arg("height"), py::arg("ppmFileName"), 
         py::arg("walls_color") = Color::gray)
    .def(py::init([](double radius, const std::string& ppmFileName, const Color &wallsColor = Color::gray) {
           auto w = std::make_unique<WorldWithTexturedGround>(radius, wallsColor, loadTexture(ppmFileName));
           w->takeObjectOwnership = false;
           return w;
         }),
         py::arg("radius"), py::arg("ppmFileName"), py::arg("walls_color") = Color::gray)
	;

}

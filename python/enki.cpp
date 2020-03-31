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

// See https://www.boost.org/doc/libs/1_72_0/libs/python/doc/html/tutorial/tutorial/exposing.html

// TODO(Jerome): Thymio -> add option to get aseba units, i.e. integers and asebaSpeed = speed * 500. / 16.6;
// TODO(Jerome): Maybe, switch to m from cm.


#include <Python.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/variant.hpp>
#include "../enki/Types.h"
#include "../enki/Geometry.h"
#include "../enki/PhysicalEngine.h"
#include "../enki/robots/e-puck/EPuck.h"
#include "../enki/robots/thymio2/Thymio2.h"
#include "../enki/robots/marxbot/Marxbot.h"
#include "../viewer/Viewer.h"
#include <QApplication>
#include <QImage>
#include <QGLWidget>

#if PY_MAJOR_VERSION >= 3
#define INT_CHECK PyLong_Check
#else
#define INT_CHECK PyInt_Check
#endif

using namespace boost::python;
using namespace Enki;

typedef boost::variant<int, double> number;

struct aseba_type_exception : std::exception
{
  char const* what() const throw() { return "Aseba units must be integers"; }
};

void translate(aseba_type_exception const& e)
{
    PyErr_SetString(PyExc_TypeError, e.what());
}

struct number_to_python
{
  static PyObject* convert(const number& value)
  {
    if(value.which() == 0)
    {
      return incref(long_(boost::get<int>(value)).ptr());
    }
    else
    {
      return incref(PyFloat_FromDouble(boost::get<double>(value)));
    }

  }
};

int to_aseba_integer(double value){
  return static_cast<int>(value);
}

double to_aseba_unit(double enki_cm){
  return enki_cm * 500 / 16.6;
}

double from_aseba_unit(double aseba_unit){
  return aseba_unit * 16.6 / 500;
}

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
  color.components[0] = extract<double>(values[0]);
  color.components[1] = extract<double>(values[1]);
  color.components[2] = extract<double>(values[2]);
  color.components[3] = extract<double>(values[3]);
}

#define def_readwrite_by_value(name, target) \
  add_property(\
    (name), \
    make_getter((target), return_value_policy<return_by_value>()), \
    make_setter((target), return_value_policy<return_by_value>()) \
  )

// vector convertion

struct Vector_to_python_tuple
{
  static PyObject* convert(const Vector& value)
  {
    return incref(make_tuple(value.x, value.y).ptr());
  }
};
struct Vector_from_python
{
  Vector_from_python()
  {
    converter::registry::push_back(
      &convertible,
      &construct,
      type_id<Vector>()
    );
  }

  static void* convertible(PyObject* objPtr)
  {
    if (PyTuple_Check(objPtr))
    {
      Py_ssize_t l = PyTuple_Size(objPtr);
      if (l != 2)
        return 0;

      PyObject* item0(PyTuple_GetItem(objPtr, 0));
      assert (item0);
      if (!(PyFloat_Check(item0) || INT_CHECK(item0)))
        return 0;
      PyObject* item1(PyTuple_GetItem(objPtr, 1));
      assert (item1);
      if (!(PyFloat_Check(item1) || INT_CHECK(item1)))
        return 0;
    }
    else
    {
      Py_ssize_t l = PyObject_Length(objPtr);
      if (l != 2)
        return 0;

      PyObject* item0(PyList_GetItem(objPtr, 0));
      assert (item0);
      if (!(PyFloat_Check(item0) || INT_CHECK(item0)))
        return 0;
      PyObject* item1(PyList_GetItem(objPtr, 1));
      assert (item1);
      if (!(PyFloat_Check(item1) || INT_CHECK(item1)))
        return 0;
    }

    return objPtr;
  }

  static void construct(PyObject* objPtr, converter::rvalue_from_python_stage1_data* data)
  {
    double x,y;

    if (PyTuple_Check(objPtr))
    {
      x = PyFloat_AsDouble(PyTuple_GetItem(objPtr, 0));
      y = PyFloat_AsDouble(PyTuple_GetItem(objPtr, 1));
    }
    else
    {
      x = PyFloat_AsDouble(PyList_GetItem(objPtr, 0));
      y = PyFloat_AsDouble(PyList_GetItem(objPtr, 1));
    }

    void* storage = ((converter::rvalue_from_python_storage<Vector>*)data)->storage.bytes;
    new (storage) Vector(x,y);
    data->convertible = storage;
  }
};

// wrappers for world

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
  QImage gt(QGLWidget::convertToGLFormat(QImage(fileName.c_str())));

  #if QT_VERSION >= QT_VERSION_CHECK(4,7,0)
  return World::GroundTexture(gt.width(), gt.height(), (const uint32_t*)gt.constBits());
  #else
  return World::GroundTexture(gt.width(), gt.height(), (uint32_t*)gt.bits());
  #endif
}

struct WorldWithoutObjectsOwnership: public World
{
  WorldWithoutObjectsOwnership(double width, double height, const Color& wallsColor = Color::gray, const GroundTexture& groundTexture = GroundTexture()):
    World(width, height, wallsColor, groundTexture)
  {
    takeObjectOwnership = false;
  }

  WorldWithoutObjectsOwnership(double r, const Color& wallsColor = Color::gray, const GroundTexture& groundTexture = GroundTexture()):
    World(r, wallsColor, groundTexture)
  {
    takeObjectOwnership = false;
  }

  WorldWithoutObjectsOwnership()
  {
    takeObjectOwnership = false;
  }
};

struct WorldWithTexturedGround: public WorldWithoutObjectsOwnership
{
  WorldWithTexturedGround(double width, double height, const std::string& ppmFileName, const Color& wallsColor = Color::gray):
    WorldWithoutObjectsOwnership(width, height, wallsColor, loadTexture(ppmFileName))
  {
  }

  WorldWithTexturedGround(double r, const std::string& ppmFileName, const Color& wallsColor = Color::gray):
    WorldWithoutObjectsOwnership(r, wallsColor, loadTexture(ppmFileName))
  {
  }
};

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

struct MarxbotWrap: Marxbot, wrapper<Marxbot>
{
  MarxbotWrap():
    Marxbot()
  {}

  virtual void controlStep(double dt)
  {
    if (override controlStep = this->get_override("controlStep"))
      controlStep(dt);

    Marxbot::controlStep(dt);
  }

};


struct EPuckWrap: EPuck, wrapper<EPuck>
{
  EPuckWrap():
    EPuck(CAPABILITY_BASIC_SENSORS|CAPABILITY_CAMERA)
  {}

  virtual void controlStep(double dt)
  {
    if (override controlStep = this->get_override("controlStep"))
      controlStep(dt);

    EPuck::controlStep(dt);
  }

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
};

struct IRCommEventWrap
{

  list intensities;
  list payloads;
  int rx_value;

  IRCommEventWrap(IRCommEvent *event)
  {
    rx_value = event->rx_value;
    for (std::vector<int>::iterator iter = event->intensities.begin(); iter != event->intensities.end(); ++iter) {
      intensities.append(*iter);
    }
    for (std::vector<int>::iterator iter = event->payloads.begin(); iter != event->payloads.end(); ++iter) {
      payloads.append(*iter);
    }
  }
};

struct Thymio2Wrap: Thymio2, wrapper<Thymio2>
{
  bool use_aseba_units;
  Thymio2Wrap(bool use_aseba_units=false):
    Thymio2(),
    use_aseba_units(use_aseba_units)
  {}

  virtual void controlStep(double dt)
  {
    if (override controlStep = this->get_override("controlStep"))
      controlStep(dt);

    Thymio2::controlStep(dt);
  }

  //From https://github.com/aseba-community/aseba/blob/bceb8ace1cb4f91d54e1520ab6b6fee5c6b9773a/aseba/targets/playground/robots/thymio2/Thymio2.cpp#L329

  double getSaturatedProxHorizontal(unsigned i)
  {
    const IRSensor* sensor(nullptr);
    switch (i)
    {
      case 0: sensor = &infraredSensor0; break;
      case 1: sensor = &infraredSensor1; break;
      case 2: sensor = &infraredSensor2; break;
      case 3: sensor = &infraredSensor3; break;
      case 4: sensor = &infraredSensor4; break;
      case 5: sensor = &infraredSensor5; break;
      case 6: sensor = &infraredSensor6; break;
      default: break;
    }
    assert(sensor);

    double dist(0);
    for (unsigned j = 0; j < sensor->getRayCount(); ++j)
      dist += sensor->getRayDist(j);
    dist /= sensor->getRayCount();
    if (dist >= sensor->getRange() - 1e-4)
      return 0;
    return sensor->getValue();
  }

  number to_number(double value)
  {
    if(use_aseba_units) return number(to_aseba_integer(value));
    return number(value);
  }

  double from_number(number value)
  {
    if(use_aseba_units && value.which() != 0)
    {
      throw aseba_type_exception();
    }
    if(use_aseba_units || value.which() == 0) return double(boost::get<int>(value));
    return boost::get<double>(value);
  }

  number to_space_number(double value)
  {
    if(use_aseba_units) return number(to_aseba_integer(to_aseba_unit(value)));
    return number(value);
  }

  double from_space_number(number value)
  {
    double r = from_number(value);
    if(use_aseba_units) r = from_aseba_unit(r);
    return r;
  }

  list getProxSensorValues(void)
  {
    list l;
    for (size_t i = 0; i < 7; i++) {
      l.append(to_number(getSaturatedProxHorizontal(i)));
    }
    return l;
  }

  // list getProxSensorValues(void)
  // {
  //   list l;
  //   l.append(infraredSensor0.getValue());
  //   l.append(infraredSensor1.getValue());
  //   l.append(infraredSensor2.getValue());
  //   l.append(infraredSensor3.getValue());
  //   l.append(infraredSensor4.getValue());
  //   l.append(infraredSensor5.getValue());
  //   l.append(infraredSensor6.getValue());
  //   return l;
  // }

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
    l.append(to_number(groundSensor0.getValue()));
    l.append(to_number(groundSensor1.getValue()));
    return l;
  }

  list getIRCommEvents(void)
  {
    list l;
    // std::vector<IRCommEvent> events = irComm.get_events();
    // for (std::vector<IRCommEvent>::iterator iter = events.begin(); iter != events.end(); ++iter) {
    //   l.append(*iter);
    // }
    std::vector<IRCommEvent> events = irComm.get_events();
    for (std::vector<IRCommEvent>::iterator iter = events.begin(); iter != events.end(); ++iter) {
      l.append(IRCommEventWrap(&(*iter)));
    }
    return l;
  }

  void setEnableIRComm(bool value)
  {
    irComm.set_enable(value);
  }

  bool getEnableIRComm(bool value)
  {
    return irComm.get_enable();
  }

  void setIRCommTx(int value)
  {
    irComm.set_tx(value);
  }

  int getIRCommTx()
  {
    return irComm.get_tx();
  }

  Color rgb(number red, number green, number blue)
  {
    double r = from_number(red);
    double g = from_number(green);
    double b = from_number(blue);
    if(use_aseba_units)
    {
      r /= 32.0;
      g /= 32.0;
      b /= 32.0;
    }
    return Color(r, g, b);
  }

  double color_value(number value)
  {
    double r = from_number(value);
    if(use_aseba_units) return r/32.0;
    return r;
  }

  void setTopBodyLedColor(number red = 0, number green = 0, number blue = 0)
  {
    setLedColor(LedIndex::TOP, rgb(red, green, blue));
  }
  void setBottomLeftBodyLedColor(number red, number green, number blue)
  {
    setLedColor(LedIndex::BOTTOM_LEFT, rgb(red, green, blue));
  }
  void setBottomRightBodyLedColor(number red, number green, number blue)
  {
    setLedColor(LedIndex::BOTTOM_RIGHT, rgb(red, green, blue));
  }

  void setButtonLedColor(unsigned int index, number value)
  {
    switch (index) {
      case 0:
        return setLedIntensity(LedIndex::BUTTON_UP, color_value(value));
      case 1:
        return setLedIntensity(LedIndex::BUTTON_RIGHT, color_value(value));
      case 2:
        return setLedIntensity(LedIndex::BUTTON_DOWN, color_value(value));
      case 3:
        return setLedIntensity(LedIndex::BUTTON_LEFT, color_value(value));
    }
  }

  void setRingLedColor(unsigned int index, number value)
  {
    // Should replicate Asebaplayground and Thymio firmware.
    if(index >=0  && index <= (LedIndex::RING_7 - LedIndex::RING_0))
    {
      return setLedIntensity(LedIndex(LedIndex::RING_0 + index), color_value(value));
    }
  }

  void setLeftRedLed(number value)
  {
    return setLedIntensity(LedIndex::LEFT_RED, color_value(value));
  }

  void setLeftBlueLed(number value)
  {
    return setLedIntensity(LedIndex::LEFT_BLUE, color_value(value));
  }

  void setRightRedLed(number value)
  {
    return setLedIntensity(LedIndex::RIGHT_RED, color_value(value));
  }

  void setRightBlueLed(number value)
  {
    return setLedIntensity(LedIndex::RIGHT_BLUE, color_value(value));
  }

  void setIRLedColor(unsigned int index, number value)
  {
    if(index >=0  && index <= (LedIndex::IR_BACK_1 - LedIndex::IR_FRONT_0))
    {
      return setLedIntensity(LedIndex(LedIndex::IR_FRONT_0 + index), color_value(value));
    }
  }


  number getLeftSpeed()
  {
    return to_space_number(leftSpeed);
  }

  void setLeftSpeed(number value)
  {
    leftSpeed = from_space_number(value);
  }

  number getRightSpeed()
  {
    return to_space_number(rightSpeed);
  }

  void setRightSpeed(number value)
  {
    rightSpeed = from_space_number(value);
  }

  number getLeftEncoder()
  {
    return to_space_number(leftEncoder);
  }

  number getRightEncoder()
  {
    return to_space_number(rightEncoder);
  }

  number getLeftOdometry()
  {
    return to_space_number(leftOdometry);
  }

  number getRightOdometry()
  {
    return to_space_number(rightOdometry);
  }


};




struct PythonViewer: public ViewerWidget
{
  PyThreadState *pythonSavedState;
  bool run_world_update;

  // PythonViewer(World& world, bool _run_world_update=false, Vector camPos=Vector(0.0, 0.0), double camAltitude=0.0, double camYaw=0.0, double camPitch=0.0, double _wallsHeight=10.0);

  PythonViewer(World& world, bool _run_world_update=false, Vector camPos=Vector(0.0, 0.0), double camAltitude=0.0, double camYaw=0.0, double camPitch=0.0, double _wallsHeight=10.0):
    ViewerWidget(&world, 0),
    pythonSavedState(0)
  {
    run_world_update = _run_world_update;
    camera.pos.setX(camPos.x);
    camera.pos.setY(camPos.y);
    camera.altitude = camAltitude;
    camera.yaw = camYaw;
    camera.pitch = camPitch;
    wallsHeight = _wallsHeight;
    managedObjectsAliases[&typeid(EPuckWrap)] = &typeid(EPuck);
    managedObjectsAliases[&typeid(MarxbotWrap)] = &typeid(Marxbot);
    managedObjectsAliases[&typeid(Thymio2Wrap)] = &typeid(Thymio2);
    setWindowTitle("PyEnki Viewer");
  }

  Vector get_camera_position()
  {
    return Vector(camera.pos.x(), camera.pos.y());
  }

  void set_camera_position(Vector value)
  {
    camera.pos.setX(value.x);
    camera.pos.setY(value.y);
  }

  double get_camera_altitude()
  {
    return camera.altitude;
  }

  void set_camera_altitude(double value)
  {
    camera.altitude = value;
  }

  double get_camera_yaw()
  {
    return camera.yaw;
  }

  void set_camera_yaw(double value)
  {
    camera.yaw = value;
  }

  double get_camera_pitch()
  {
    return camera.pitch;
  }

  void set_camera_pitch(double value)
  {
    camera.pitch = value;
  }

  void timerEvent(QTimerEvent * event)
  {
    // get back Python lock
    if (pythonSavedState)
      PyEval_RestoreThread(pythonSavedState);
    // touch Python objects while locked
    // ViewerWidget::timerEvent(event);
    if(run_world_update) world->step(double(timerPeriodMs)/1000., 3);
    updateGL();
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
  PythonViewer viewer(world, true, camPos, camAltitude, camYaw, camPitch, wallsHeight);
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

double float_value(number const& value)
{
    return boost::get<double>(value);
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(step_overloads, step, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(runInViewer_overloads, runInViewer, 1, 6)

BOOST_PYTHON_MODULE(pyenki)
{
  register_exception_translator<aseba_type_exception>(&translate);
  // implicitly_convertible<number,double>();
  implicitly_convertible<int,number>();
  implicitly_convertible<float,number>();
  to_python_converter<number, number_to_python>();

  // implicitly_convertible<number,double>();
  // implicitly_convertible<number,int>();
  // implicitly_convertible<int,number>();

  // setup converters
  to_python_converter<Vector, Vector_to_python_tuple>();
  Vector_from_python();


  // .def("__init__", bp::make_constructor(&Net_Init,
  //           bp::default_call_policies(), (bp::arg("network_file"), "phase",
  //             bp::arg("level")=0, bp::arg("stages")=bp::object(),
  //             bp::arg("weights_file")=bp::object())))

  class_<PythonViewer, boost::noncopyable>(
      "WorldView",
      init<World&, bool, Vector, double, double, double, double>
      ((arg("world"), arg("run_world_update")=false, arg("cam_position")=Vector(0.0, 0.0), arg("cam_altitude")=0.0,
       arg("cam_yaw")=0.0, arg("cam_pitch")=0.0, arg("walls_height")=10.0)))
      // (args("world", "run_world_update", "cam_position", "cam_altitude", "cam_yaw", "cam_pitch", "walls_height")))
  // .def("update", &PythonViewer::updateGL)
  .def("show", &PythonViewer::show)
  .def("hide", &PythonViewer::hide)
  .add_property("cam_position", &PythonViewer::get_camera_position, &PythonViewer::set_camera_position)
  .add_property("cam_altitude", &PythonViewer::get_camera_altitude, &PythonViewer::set_camera_altitude)
  .add_property("cam_yaw", &PythonViewer::get_camera_yaw, &PythonViewer::set_camera_yaw)
  .add_property("cam_pitch", &PythonViewer::get_camera_pitch, &PythonViewer::set_camera_pitch)
  .def_readwrite("run_world_update", &PythonViewer::run_world_update)
  // .add_property("tracking", &PythonViewer::isTrackingActivated, &PythonViewer::setTracking)
  ;

  // TODO: complete doc

  // Color and texture

  class_<Color>("Color",
    "A color in RGBA",
    init<optional<double, double, double, double> >(
      "Create a RGBA color.\n\n"
      "Arguments:\n"
      "    r -- red component [0..1], default: 0.0\n"
      "    g -- green component [0..1], default: 0.0\n"
      "    b -- blue component [0..1], default: 0.0\n"
      "    a -- alpha (transparency) component [0..1], default: 1.0\n",
      args("r", "g", "b", "a")
    )
  )
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
    .def(self_ns::str(self_ns::self))
    .def("threshold", &Color::threshold)
    .def("to_gray", &Color::toGray)
    .def_readonly("black", &Color::black)
    .def_readonly("white", &Color::white)
    .def_readonly("gray", &Color::gray)
    .def_readonly("red", &Color::red)
    .def_readonly("green", &Color::green)
    .def_readonly("blue", &Color::blue)
    .add_property("r", &Color::r, &Color::setR)
    .add_property("g", &Color::g, &Color::setG)
    .add_property("b", &Color::b, &Color::setB)
    .add_property("a", &Color::a, &Color::setA)
    .add_property("components", getColorComponents, setColorComponents)
  ;

  class_<Texture>("Texture")
    .def(vector_indexing_suite<Texture>())
  ;

  class_<Textures>("Textures")
    .def(vector_indexing_suite<Textures>())
  ;

  // Physical objects

  class_<PhysicalObject>("PhysicalObject", no_init)
    .def_readonly("radius", &PhysicalObject::getRadius)
    .def_readonly("height", &PhysicalObject::getHeight)
    .def_readonly("is_cylindric", &PhysicalObject::isCylindric)
    .def_readonly("mass", &PhysicalObject::getMass)
    .def_readonly("moment_of_inertia", &PhysicalObject::getMomentOfInertia)
    .def_readonly("_interlaced_distance", &PhysicalObject::getInterlacedDistance)
    .def_readwrite("collision_elasticity", &PhysicalObject::collisionElasticity)
    .def_readwrite("dry_friction_coefficient", &PhysicalObject::dryFrictionCoefficient)
    .def_readwrite("viscous_friction_coefficient", &PhysicalObject::viscousFrictionCoefficient)
    .def_readwrite("viscous_moment_friction_coefficient", &PhysicalObject::viscousMomentFrictionCoefficient)
    .def_readwrite_by_value("position", &PhysicalObject::pos)
    .def_readwrite("angle", &PhysicalObject::angle)
    .def_readwrite_by_value("velocity", &PhysicalObject::speed)
    .def_readwrite("angular_speed", &PhysicalObject::angSpeed)
    .add_property("color",  make_function(&PhysicalObject::getColor, return_value_policy<copy_const_reference>()), &PhysicalObject::setColor)
    // warning setting the "color" property at run time using the viewer from the non-gui thread will lead to a crash because it will do an OpenGL call from that thread
  ;

  class_<CircularPhysicalObject, bases<PhysicalObject> >("CircularObject",
    init<double, double, double, optional<const Color&> >(args("radius", "height", "mass", "color"))
  );

  class_<RectangularPhysicalObject, bases<PhysicalObject> >("RectangularObject",
    init<double, double, double, double, optional<const Color&> >(args("l1", "l2", "height", "mass", "color"))
  );

  // Robots

  class_<Robot, bases<PhysicalObject> >("PhysicalObject", no_init)
  ;

  class_<DifferentialWheeled, bases<Robot> >("DifferentialWheeled", no_init)
    .def_readwrite("left_wheel_speed", &DifferentialWheeled::leftSpeed)
    .def_readwrite("right_wheel_speed", &DifferentialWheeled::rightSpeed)
    .def_readonly("left_wheel_encoder", &DifferentialWheeled::leftEncoder)
    .def_readonly("right_wheel_encoder", &DifferentialWheeled::rightEncoder)
    .def_readonly("left_wheel_odometry", &DifferentialWheeled::leftOdometry)
    .def_readonly("right_wheel_odometry", &DifferentialWheeled::rightOdometry)
    .def("reset_encoders", &DifferentialWheeled::resetEncoders)
  ;

  class_<MarxbotWrap, bases<DifferentialWheeled>, boost::noncopyable>("Marxbot")
    .def("controlStep", &EPuckWrap::controlStep)
  ;

  class_<EPuckWrap, bases<DifferentialWheeled>, boost::noncopyable>("EPuck")
    .def("controlStep", &EPuckWrap::controlStep)
    .def_readonly("proximity_sensor_values", &EPuckWrap::getProxSensorValues)
    .def_readonly("proximity_sensor_distances", &EPuckWrap::getProxSensorDistances)
    .def_readonly("camera_image", &EPuckWrap::getCameraImage)
  ;

  // typedef std::vector<int> MyList;
  //
  // class_<MyList>("list")
  //   .def(vector_indexing_suite<MyList>() );
  // class_<IRCommEvent>("IRCommEvent", no_init)
  //   .def_readonly("rx_value", &IRCommEvent::rx_value)
  //   .def_readonly("intensities", &IRCommEvent::intensities)
  //   .def_readonly("payloads", &IRCommEvent::payloads)
  // ;

  class_<IRCommEventWrap>("IRCommEvent", no_init)
    .def_readonly("rx", &IRCommEventWrap::rx_value)
    .def_readonly("intensities", &IRCommEventWrap::intensities)
    .def_readonly("payloads", &IRCommEventWrap::payloads)
  ;

  class_<Thymio2Wrap, bases<DifferentialWheeled>, boost::noncopyable>("Thymio2", "",
  init<optional<bool>>(
      "Create a Thymio2 robot.\n\n"
      "Arguments:\n"
      "    use_aseba_units -- use the same units and type as Aseba, default: False\n",
      args("use_aseba_units")))
    .def("controlStep", &Thymio2Wrap::controlStep)
    .def_readonly("prox_values", &Thymio2Wrap::getProxSensorValues)
    .def_readonly("prox_distances", &Thymio2Wrap::getProxSensorDistances)
    .def_readonly("ground_values", &Thymio2Wrap::getGroundSensorValues)
    .def_readonly("prox_comm_events", &Thymio2Wrap::getIRCommEvents)
    .add_property("prox_comm_tx", &Thymio2Wrap::getIRCommTx, &Thymio2Wrap::setIRCommTx)
    .add_property("prox_comm_enable", &Thymio2Wrap::getEnableIRComm, &Thymio2Wrap::setEnableIRComm)
    .def("set_led_top", &Thymio2Wrap::setTopBodyLedColor, ( arg("red")=0, arg("green")=0, arg("blue")=0))
    .def("set_led_bottom_left", &Thymio2Wrap::setBottomLeftBodyLedColor, ( arg("red")=0, arg("green")=0, arg("blue")=0))
    .def("set_led_bottom_right", &Thymio2Wrap::setBottomRightBodyLedColor, ( arg("red")=0, arg("green")=0, arg("blue")=0))
    .def("set_led_buttons", &Thymio2Wrap::setButtonLedColor)
    .def("set_led_circle", &Thymio2Wrap::setRingLedColor)
    .def("set_led_prox", &Thymio2Wrap::setIRLedColor)
    .def("set_led_left_red", &Thymio2Wrap::setLeftRedLed)
    .def("set_led_left_blue", &Thymio2Wrap::setLeftBlueLed)
    .def("set_led_right_red", &Thymio2Wrap::setRightRedLed)
    .def("set_led_right_blue", &Thymio2Wrap::setRightBlueLed)
    .add_property("motor_left_target", &Thymio2Wrap::getLeftSpeed, &Thymio2Wrap::setLeftSpeed)
    .add_property("motor_right_target", &Thymio2Wrap::getRightSpeed, &Thymio2Wrap::setRightSpeed)
    .add_property("motor_left_speed", &Thymio2Wrap::getLeftEncoder)
    .add_property("motor_right_speed", &Thymio2Wrap::getRightEncoder)
    .add_property("motor_left_odometry", &Thymio2Wrap::getLeftOdometry)
    .add_property("motor_right_odometry", &Thymio2Wrap::getRightOdometry)
    .def_readwrite_by_value("use_aseba_units", &Thymio2Wrap::use_aseba_units)
  ;

  // World




  class_<World>("WorldBase", no_init)
  ;

  class_<WorldWithoutObjectsOwnership, bases<World> >("World",
    "The world is the container of all objects and robots.\n"
    "It is either a rectangular arena with walls at all sides, a circular area with walls, or an infinite surface."
    ,
    init<double, double, optional<const Color&> >(args("width", "height", "walls_color"))
  )
    .def(init<double, optional<const Color&> >(args("r", "walls_color")))
    .def(init<>())
    .def("step", &World::step, step_overloads(args("dt", "physics_oversampling")))
    .def("add_object", &World::addObject, with_custodian_and_ward<1,2>())
    .def("remove_object", &World::removeObject)
    .def("set_random_seed", &World::setRandomSeed)
    .def("run", run)
    .def("run_in_viewer", runInViewer, runInViewer_overloads(args("self", "cam_position", "cam_altitude", "cam_yaw", "cam_pitch", "walls_height")))
  ;

  class_<WorldWithTexturedGround, bases<WorldWithoutObjectsOwnership> >("WorldWithTexturedGround",
    init<double, double, const std::string&, optional<const Color&> >(args("width", "height", "ppm_filename", "walls_color"))
  )
    .def(init<double, const std::string&, optional<const Color&> >(args("r", "ppm_filename", "walls_color")))
  ;

}

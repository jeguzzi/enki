#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "../enki/PhysicalEngine.h"

class PyWorld;

using Termination = std::function<bool(const PyWorld &)>;
using Callback = std::function<void(PyWorld &)>;

namespace py = pybind11;

namespace pybind11 {
namespace detail {
template <> struct type_caster<Enki::Vector> {

  PYBIND11_TYPE_CASTER(Enki::Vector, const_name("Vector"));

  static handle cast(const Enki::Vector &src, return_value_policy policy,
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
} // namespace detail
} // namespace pybind11

struct PyWorld : public Enki::World {

  std::optional<py::object> numpy_rng;

  PyWorld(double width, double height,
          const Enki::Color &wallsColor = Enki::Color::gray,
          const std::optional<GroundTexture> & groundTexture = std::nullopt,
          unsigned long seed = 0)
      : Enki::World(width, height, wallsColor,
                    groundTexture.value_or(GroundTexture()), seed) {
    takeObjectOwnership = false;
  }

  PyWorld(double radius, const Enki::Color &wallsColor = Enki::Color::gray,
          const std::optional<GroundTexture> & groundTexture = std::nullopt,
          unsigned long seed = 0)
      : Enki::World(radius, wallsColor, groundTexture.value_or(GroundTexture()),
                    seed) {
    takeObjectOwnership = false;
  }

  PyWorld(unsigned long seed = 0) : Enki::World(seed) {
    takeObjectOwnership = false;
  }

  void setRandomSeed(unsigned long seed) {
    if (seed != getRandomSeed()) {
      py::module_ np = py::module_::import("numpy");
      numpy_rng = np.attr("random").attr("default_rng")(seed);
    }
    Enki::World::setRandomSeed(seed);
  }

  void setRandom(py::object value) { numpy_rng = value; }

  py::object getRandom() {
    if (!numpy_rng) {
      py::module_ np = py::module_::import("numpy");
      numpy_rng = np.attr("random").attr("default_rng")(getRandomSeed());
    }
    return *numpy_rng;
  }

  void copyRandom(PyWorld &world) {
    Enki::World::copyRandom(static_cast<Enki::World &>(world));
    setRandom(world.getRandom());
  }

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
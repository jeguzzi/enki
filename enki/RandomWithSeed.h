#ifndef __ENKI_RANDOM_NEW_H
#define __ENKI_RANDOM_NEW_H

// Jerome Guzzi: Modified Random.h so that sampling is deterministic
// and reproducible when setting a seed.
// 
// A Random instance should belong to each world and all sampling should 
// happen through the instance.

#include <random>

namespace Enki {
//! A fast random generator

class Random {
private:
  unsigned long _randx; //!< value used to compute next pseudo-random value
  unsigned long _seed;
  std::uniform_real_distribution<double> _dist;
  std::mt19937 _gen;

public:
  //! Construct the random generator, initialize with a seed of 0
  explicit Random(unsigned long seed = 0) : _randx(seed), _seed(seed), _gen(seed) {}
  //! Set the seed
  void setSeed(unsigned long seed) {
    _randx = seed;
    _seed = seed;
    _gen.seed(seed);
  }
  unsigned long getSeed() const { return _seed; }
  //! Get a random number between 0 and 2^31
  unsigned long get() {
    return (_randx = _randx * 1103515245 + 12345) & 0x7fffffff;
  }
  //! Get a random double between 0 and range, use get() internally
  double getRange(double range) {
    return (static_cast<double>(get()) * range) / 2147483648.0;
  }

  //! Return a number in [0;1[ in a uniform distribution
  /*! \ingroup an */
  double uniformRand() { return _dist(_gen); }

  // Jerome Guzzi: replaces UniformRand(...)().
  double uniformRange(double from = 0.0, double to = 1.0) {
    return from + (to - from) * uniformRand();
  }

  //! Return a random number with a gaussian distribution of a certain mean and
  //! standard deviation.
  /*! \ingroup an */
  double gaussianRand(double mean, double sigm) {
    // Generation using the Polar (Box-Mueller) method.
    // Code inspired by GSL, which is a really great math lib.
    // http://sources.redhat.com/gsl/
    // C++ wrapper available.
    // http://gslwrap.sourceforge.net/
    double r, x, y;

    // Generate random number in unity circle.
    do {
      x = uniformRand() * 2 - 1;
      y = uniformRand() * 2 - 1;
      r = x * x + y * y;
    } while (r > 1.0 || r == 0);

    // Box-Muller transform.
    return sigm * y * sqrt(-2.0 * log(r) / r) + mean;
  }
};
} // namespace Enki

#endif

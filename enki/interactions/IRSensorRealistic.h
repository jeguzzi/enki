/*
Author: Jerome Guzzi
*/

#ifndef __ENKI_IRSENSOR_REALISTIC_H
#define __ENKI_IRSENSOR_REALISTIC_H

#include <enki/interactions/IRSensor.h>

namespace Enki {
// TODO: Document
class IRSensorRealistic : public IRSensor {
public:
  IRSensorRealistic(Robot *owner, Vector pos, double height, double orientation,
                    double range, double m, double x0, double c,
                    double noiseSd = 0., unsigned int rays = 3,
                    double _aperture = 15.0, double k = 6.0,
                    double min_value = 1000.);
  void finalize(double dt, World *w);

protected:
  void updateRay(size_t i, double dist);
  std::vector<double> ray_weights;
  double ray_weights_sum;
  unsigned int min_value;
  double alpha;
};

} // namespace Enki

#endif // __ENKI_IRSENSOR_REALISTIC_H

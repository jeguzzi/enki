/*
Author: Jerome Guzzi
*/

#include "IRSensorRealistic.h"

namespace Enki {

IRSensorRealistic::IRSensorRealistic(Robot *owner, Vector pos, double height,
                                     double orientation, double range, double m,
                                     double x0, double c, double noiseSd,
                                     unsigned int rays, double _aperture,
                                     double k, double min_value)
    : IRSensor(owner, pos, height, orientation, range, m, x0, c, noiseSd, rays,
               _aperture),
      min_value(min_value) {
  ray_weights_sum = 0.0;
  ray_weights.resize(rays);
  for (size_t i = 0; i < rays; i++) {
    ray_weights[i] = pow(cos(rayAngles[i]), k);
    ray_weights_sum += ray_weights[i];
  }
  alpha = aperture / ((rays - 1) / 2);
}

void IRSensorRealistic::finalize(double dt, World *w) {
  double s = 0.0;
  // z = cos(beta) where beta is the angle between ray and obstacle (on both
  // side of the intersection)
  double zs[2 * rayAngles.size()];
  for (size_t i = 0; i < rayAngles.size() - 1; i++) {
    double a = rayDists[i];
    double b = rayDists[i + 1];
    if (a > range || b > range) {
      zs[2 * i + 1] = 0;
      zs[2 * i + 2] = 0;
    } else {
      // length of the obstacle segment delimited by the two rays
      double _c = sqrt(a * a + b * b - 2 * cos(alpha) * a * b);
      // This would be the mid ray z
      // double z = sin(alpha / 2) * (a + b) / c;
      zs[2 * i + 2] = sin(alpha) * a / _c;
      zs[2 * i + 1] = sin(alpha) * b / _c;
    }
  }
  zs[0] = 0;
  zs[2 * rayAngles.size() - 1] = 0;

  for (size_t i = 0; i < rayAngles.size(); i++) {
    double dx = rayDists[i] - x0;
    if (dx > 0) {
      s += ray_weights[i] * 0.5 / (dx * dx) *
           std::max(zs[2 * i + 1], zs[2 * i + 0]);
    } else {
      s += 1e6;
      break;
    }
  }
  if (s == 0.0) {
    finalValue = 0;
  } else {
    s /= ray_weights_sum;
    double d = 1.0 + 1.0 / s / (c - x0 * x0);
    if (d > 1.0) {
      finalValue = m / d;
    } else {
      d = m;
    }
  }
  finalValue = std::max(0., std::min(m, w->getRandom().gaussianRand(finalValue, noiseSd)));
  if (finalValue < min_value) {
    finalValue = 0.0;
    finalDist = range;
  } else {
    finalDist = inverseResponseFunction(finalValue);
  }
}

void IRSensorRealistic::updateRay(size_t i, double dist) {
  if (dist < rayDists[i]) {
    rayDists[i] = dist;
  }
}

} // namespace Enki

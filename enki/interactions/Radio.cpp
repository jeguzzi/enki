/*
Author: Jerome Guzzi
*/

#include <enki/Geometry.h>
#include <enki/interactions/Radio.h>
#include <iostream>
#include <map>
#include <vector>

namespace Enki {
bool CircularSector::contains(Point &p) {
  Vector dp = p - center;
  if (dp.norm() > radius)
    return false;
  double angle = dp.angle();
  if (normalizeAngle(angle - begin) < 0 || normalizeAngle(angle - end) > 0)
    return false;
  return true;
}

std::ostream &operator<<(std::ostream &os, CircularSector const &sector) {
  return os << "(" << sector.center.x << ", " << sector.center.y << ") "
            << sector.radius << " [" << sector.begin << ", " << sector.end
            << "] " << sector.angle;
}
} // namespace Enki

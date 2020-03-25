#include <map>
#include <vector>
#include <enki/interactions/Radio.h>
#include <enki/Geometry.h>
// #include <iostream>

namespace Enki
{

  bool CircularSector::contains(Point &p){
      Vector dp = p - center;
      // std::cout << "D:" << dp.norm() << " <" << radius << "?\n";
      // if (abs(dp.norm() - radius) < 500)
      // {
      //   std::cout << dp.norm() << " " << radius << " " << (dp.norm() - radius) << "\n";
      // }

      if (dp.norm() > radius) return false;
      double angle = dp.angle();
      // std::cout << "angle " << angle << " in ?" << begin << " - " << end << "\n";
      // std::cout << "A1: " << normalizeAngle(angle - begin) << " >0?\n";
      // std::cout << "A2: " << normalizeAngle(angle - end) << " <0?\n";
      if (normalizeAngle(angle - begin) < 0 || normalizeAngle(angle - end) > 0) return false;
      return true;
    }

  std::ostream &operator<<(std::ostream &os, CircularSector const &sector)
  {
    return os << "(" << sector.center.x << ", " << sector.center.y << ") " << sector.radius << " [" << sector.begin << ", " << sector.end << "]";
  }

  // template<typename T>
  // std::ostream &operator<<(std::ostream &os, RadioMessage<T> const &msg) {
  //     return os << msg.source_uid << " -> " << msg.data << " in " << msg.sectors;
  // }

}

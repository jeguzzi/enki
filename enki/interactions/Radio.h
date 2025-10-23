/*
Author: Jerome Guzzi
*/

#ifndef __ENKI_RADIO_H
#define __ENKI_RADIO_H

#include <enki/Geometry.h>
#include <map>
#include <vector>

namespace Enki {
struct CircularSector {
  double begin;
  double end;
  double radius;
  double angle;
  Point center;
  bool contains(Point &p);
};
std::ostream &operator<<(std::ostream &os, CircularSector const &sector);

template <typename T> struct RadioMessage {
  int source_uid;
  T data;
  std::vector<CircularSector> sectors;
};

template <typename T>
std::ostream &operator<<(std::ostream &os, RadioMessage<T> const &msg);

template <typename T> class Radio {
private:
  std::map<int, RadioMessage<T>> messages;

public:
  Radio() : messages() {}
  void send(int source_uid, RadioMessage<T> message) {
    messages[source_uid] = message;
  }
  void remove(int source_uid) { messages.erase(source_uid); }
  std::map<int, RadioMessage<T>> *get_messages() { return &messages; }
};
} // namespace Enki
#endif // __ENKI_RADIO_H

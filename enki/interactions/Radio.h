#include <map>
#include <vector>
#include <enki/Geometry.h>

#ifndef __ENKI_RADIO_H
#define __ENKI_RADIO_H

namespace Enki
{
  struct CircularSector
  {
    double begin;
    double end;
    double radius;
    double angle;
    Point center;
    bool contains(Point &p);
  };
  std::ostream &operator<<(std::ostream &os, CircularSector const &sector);


  template<typename T>
  struct RadioMessage
  {
    int source_uid;
    T data;
    std::vector<CircularSector> sectors;
  };

  template<typename T>
  std::ostream &operator<<(std::ostream &os, RadioMessage<T> const &msg);

  // TODO: radio should be a global object, not a global interaction of every robot.
  // but we could replicate the same (without modifiyng `PhysicalEngine`)
  // by checking in init and finalize if the dt step was already performced.
  template<typename T>
  class Radio
  {
  private:
    std::map<int, RadioMessage<T> > messages;
  public:
    Radio() : messages() {}
    // std::vector< RadioMessage<T> > receive(const Point& p, double orientation) { std::vector< RadioMessage<T> >(0); }
    void send(int source_uid, RadioMessage<T> message) { messages[source_uid]=message; }
    void remove(int source_uid) { messages.erase(source_uid); }
    // void init(double dt, World *w) { future_messages.clear(); }
    // void finalize(double dt, World *w) { messages = future_messages; }
    std::map<int, RadioMessage<T> >* get_messages() { return &messages; }
  };
}
#endif

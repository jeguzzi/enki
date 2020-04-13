/*

Realistic simulation of IR communication

Visibility:

Given emitter (IRSensor) at point p_e and angle a_e, receiver (IRSensor) at point p_r and angle a_r,
a pulse from e to r is visible if:

  1) the distance between e and r is less then `range` ~ 23 cm
  2) p_r is inside the circular sector [a_e - aperture, ae + aperture] centered at p_e, with `aperture` ~ 15 degrees (0.268)
  3) p_e is inside the circular sector [a_r - aperture_r, ar + aperture_r] centered at p_r, with `aperture_r` ~ 36 degrees (0.644)
  4) there are no obstacles between p_r and p_e

1-3) are easy and fast to check as they depends only on the pose of the two sensors
4) require to check collisions or to reuse the computation done by the IRSensors
In this implementation we use ray distance computed by the emitter for {-15, 0, 15}
This approximation assumes that obstacles are larger than 15 degree at 23 cm, i.e. 6 cm.

We have now added another version enabled by the definition of SUM_INTENSITIES:
the intensities of all emitters are summed up and then the response function is computed.
This is relevant only if emitter sectors overlaps, which is the case if
We assume that I = I_0/(x-x_0)^2 (where I_0 is unknown and not needed).
Then the response function for a single emitter, i.e. aperture >= delta_angle / 2 ,
which is the case [also] for the front facing thymio sensors (aperture ~ 0.268, delta_angle ~0.34906)
R = m (c - x_0^2)/(x^2 - 2xx_0 +c) = m / (((x-x_0)^2)/(c-x_0^2) + 1)
is the same as
R  = m / ((I_0/I)/(c-x_0^2) + 1)
so we assume that for many emitters I_tot = \sum I
and R = m / ((I_0/I_tot)/(c-x_0^2) + 1) = m / (1/(\sum 1/((x-x_0)^2))/(c-x_0^2) + 1)
If we actually ignore x_0 ~= 0.02 << sqrt(c) ~= 16.6:
then:
  R = m / 1/(\sum((x/c)^{-2})) + 1
Moreover, we set to 0 all responses not large enough (i.e lower than the response caused by an emitter at distance = range.
Note that this is still a simplification because we ignore that the real intensities depends also on the orientations.

TODO(Jerome): add noise to both versions

use prox model:
response = std::max(0., std::min(m, gaussianRand(response, noiseSd)));
*/

#include "IRComm.h"
#include <algorithm>
#include <assert.h>
#include <iostream>
#include <limits>
#include <sstream>

/*!	\file IRSensor.cpp
  \brief Implementation of the generic infrared sensor
*/

// #define SUM_INTENSITIES




namespace Enki {

// static std::map<World *, IRCommRadio> radios;



// CircularSector receiver_sector(IRSensor *sensor, double range, double receiver_aperture);

std::ostream &operator<<(std::ostream &os, IRCommEvent const &event) {
    os << "  - value: " << event.rx_value;
    os << "\n    payloads: [";
    for (std::vector<int>::const_iterator i = event.payloads.begin(); i != event.payloads.end(); ++i)
      os<< *i << ", ";
    os << "]";
    os << "\n    intensities: [";
    for (std::vector<int>::const_iterator i = event.intensities.begin(); i != event.intensities.end(); ++i)
      os<< *i << ", ";
    os << "]";
    return os;
}

std::ostream &operator<<(std::ostream &os, RadioMessage<int> const &msg) {
    os << msg.source_uid << " -> " << msg.data << " in\n";
    for (std::vector<CircularSector>::const_iterator i = msg.sectors.begin(); i != msg.sectors.end(); ++i)
      os<< "\t" << *i << "\n";
    return os;
}

CircularSector receiver_sector(IRSensor *sensor, double range, double receiver_aperture)
{
  CircularSector sector;
  sector.radius = range;
  sector.center = sensor->getAbsolutePosition();
  double angle = sensor->getAbsoluteOrientation();
  sector.begin = angle - receiver_aperture;
  sector.end = angle + receiver_aperture;
  return sector;
}

std::vector<CircularSector> emitter_sectors(IRSensor *sensor) {
  const double tol = 0.5;
  std::vector<CircularSector> sectors;
  for (size_t i = 0; i < sensor->getRayCount() - 1; i++) {
    CircularSector sector;
    sector.radius = std::max(sensor->getRayDist(i), sensor->getRayDist(i + 1)) + tol;
    sector.center = sensor->getAbsolutePosition();
    sector.begin = sensor->getAbsRayAngle(i);
    sector.end = sensor->getAbsRayAngle(i + 1);
    sectors.push_back(sector);
  }
  return sectors;
}

bool intersection(CircularSector &sector1, CircularSector &sector2) {
  return sector1.contains(sector2.center) && sector2.contains(sector1.center);
}

double received_ir_message_distance(IRSensor *sensor, IRMessage *message, double range, double receiver_aperture) {
  CircularSector s_sector = receiver_sector(sensor, range, receiver_aperture); // Could precompute them at init
  std::vector<CircularSector> *m_sectors = &(message->sectors);
  for (std::vector<CircularSector>::iterator it = m_sectors->begin();
       it != m_sectors->end(); it++) {
    if (intersection(*it, s_sector)) return (it->center - s_sector.center).norm();
  }
  return 0;
}

//Same response function but different params
double responseFunction(double x, double range, double m, double c, double x0)
{
  if (x < x0) return m;
  if (x > range) return 0;
  return m*(c - x0*x0)/(x*x - 2*x0*x + c);
}

// New version that sums the amplitude of the sectors
double received_ir_message_intensity(IRSensor *sensor, IRMessage *message, double range, double receiver_aperture, double m, double c, double x0, double min_response) {
  CircularSector s_sector = receiver_sector(sensor, range, receiver_aperture);
  std::vector<CircularSector> *m_sectors = &(message->sectors);
  double inverse_square_sum = 0;
  for (std::vector<CircularSector>::iterator it = m_sectors->begin();
       it != m_sectors->end(); it++) {
    if (intersection(*it, s_sector))
    {
      double dx = (it->center - s_sector.center).norm() - x0;
      inverse_square_sum += 1.0 / (dx * dx);
    }
  }
  if(inverse_square_sum)
  {
    double response = m / (1.0 + 1.0 / inverse_square_sum / (c - x0*x0));
    if(response < min_response) return 0.0;
    else return response;
  }
  return 0.0;
}

std::map<World *, IRCommRadio *> IRComm::radios = std::map<World *, IRCommRadio *>();

IRCommRadio * IRComm::radio_in_world(World * world)
{
  if(radios.count(world) == 0)
    radios[world] = new IRCommRadio();
  return radios[world];
}

void IRComm::receive_events() {
  std::map<int, IRMessage> *messages = radio->get_messages();
  for (std::map<int, IRMessage>::iterator mit = messages->begin();
       mit != messages->end(); mit++) {
    // std::cout << "test message " << (mit->second) << "\n";
    IRCommEvent event;
    int source = mit->first;
    if (source == owner->uid) continue;
    // TODO(Jerome)
    event.phase = 0;
    bool received = false;
    int i = 0;
    for (std::vector<IRSensor *>::iterator it = sensors.begin();
         it != sensors.end(); it++) {
      // std::cout << "test sensor " << i <<"\n";
      i++;
      #ifdef SUM_INTENSITIES
      if (double intensity = received_ir_message_intensity(*it, &(mit->second), range, receiver_aperture, m, c, x0, min_intensity)) {
      #else
      if (double distance = received_ir_message_distance(*it, &(mit->second), range, receiver_aperture)) {
        double intensity = responseFunction(distance, range, m, c, x0);
      #endif
        event.rx_value = mit->second.data;
        event.payloads.push_back(event.rx_value);
        intensity = std::max(0., std::min(4200.0, gaussianRand(intensity, noiseSd)));
        event.intensities.push_back(intensity);
        received = true;
      } else {
        event.payloads.push_back(0);
        event.intensities.push_back(0);
      }
    }
    if (received) {
      events.push_back(event);
    }
  }
}

void IRComm::init(double dt, World* w)
{
  time += dt;
  if(!radio) radio = radio_in_world(w);
  // std::cout << owner->uid << " IRComm::init\n";
  // std::cout << owner->uid << " IRComm::init done\n";
}

const double EPSILON = -1.0e-3;

void IRComm::step(double dt, World* w)
{
  // send messages (!after IRSensor has computed the ray intersections)
  if(enabled && time - last_sent - period > EPSILON)
  {
    if (last_sent < 0)
      last_sent = time;
    else
      last_sent += period;
    radio->send(owner->uid, message());
  }
  else{
    radio->remove(owner->uid);
  }
}

void IRComm::finalize(double dt, World* w)
{
  // receive messages after all robots/IRComm have put their messages onair.
  events.clear();
  receive_events();
}


IRMessage IRComm::message() {
  IRMessage message;
  message.data = tx_value;
  message.source_uid = owner->uid;
  for (std::vector<IRSensor *>::iterator it = sensors.begin();
       it != sensors.end(); it++) {
    std::vector<CircularSector> sectors = emitter_sectors(*it);
    message.sectors.insert(message.sectors.end(), sectors.begin(), sectors.end());
  }
  return message;
}

IRComm::~IRComm()
{
  // Cleaning up
  if(radio)
  {
    std::map<World *, IRCommRadio *>::iterator it;
    for(it = radios.begin(); it != radios.end(); it++ )
    {
      if(it->second == radio)
      {
        delete radio;
        radios.erase(it);
        return;
      }
    }
  }
}

// IRCommRadio *IRComm::radio = new IRCommRadio();
} // namespace Enki

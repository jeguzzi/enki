/*
Author: Jerome Guzzi
*/

#ifndef __ENKI_IRSENSORWITHCOMM_H
#define __ENKI_IRSENSORWITHCOMM_H

#include <enki/Geometry.h>
#include <enki/Interaction.h>
#include <enki/PhysicalEngine.h>
#include <enki/interactions/IRSensor.h>
#include <enki/interactions/Radio.h>
/*!	\file IRSensor.h
  \brief Header of the generic infrared sensor
*/

namespace Enki {
class IRComm;

typedef RadioMessage<int> IRMessage;
typedef Radio<int> IRCommRadio;

struct IRCommEvent {
  int rx_value;
  double phase;
  std::vector<int> intensities;
  std::vector<int> payloads;
};

std::ostream &operator<<(std::ostream &os, IRCommEvent const &event);

double responseFunction(double x, double range, double m, double c, double x0);

class IRComm : public GlobalInteraction {
private:
  IRMessage message();
  IRCommRadio *radio;
  static std::map<World *, IRCommRadio *> radios;
  bool enabled;
  int tx_value;
  std::vector<IRCommEvent> events;
  std::vector<IRSensor *> sensors;
  const double range;
  double last_sent;
  double time;
  const double period;
  const double receiver_aperture;

  const double m;
  //! Position of the maximum of response (might be negative, inside the robot),
  //! second parametere of response function
  const double x0;
  //! Third parameter of response function
  const double c;
  //! Standard deviation of Gaussian noise in the response space
  const double noiseSd;

  const double min_intensity;
  const double ke;
  const double kr;

  IRCommRadio *radio_in_world(World *world);

public:
  // IRComm(Robot *owner, std::vector<IRSensor *> sensors) : enabled(false),
  // sensors(sensors), tx_value(0) {};
  IRComm(Robot *owner, double range = 25, double period = 0.1,
         double aperture = 0.644, double m = 4200, double x0 = 0.02,
         double c = 275, double noiseSd = 0., double min_intensity = 1400,
         double ke = 20.0, double kr = 10.0)
      : GlobalInteraction(owner), radio(NULL), enabled(false), tx_value(0),
        range(range), last_sent(-1.0), time(0), period(period),
        receiver_aperture(aperture), m(m), x0(x0), c(c), noiseSd(noiseSd),
        min_intensity(min_intensity), ke(ke), kr(kr) {
    // min_intensity = responseFunction(range, range, m, c, x0);
  }

  void add_sensor(IRSensor *sensor) {
    sensor->setSearchRange(range);
    sensors.push_back(sensor);
  }
  void init(double dt, World *w);
  void finalize(double dt, World *w);
  void step(double dt, World *w);
  void receive_events(World *w);
  // ~IRComm() { } ;
  ~IRComm();
  void set_enable(bool value) {
    enabled = value;
    last_sent = -1.0;
  }
  bool get_enable() { return enabled; }
  void set_tx(int value) { tx_value = value; }
  int get_tx() { return tx_value; }
  std::vector<IRCommEvent> get_events() { return events; }
};
} // namespace Enki

#endif // __ENKI_IRSENSORWITHCOMM_H

/*
    Enki - a fast 2D robot simulator
    Copyright (C) 1999-2016 Stephane Magnenat <stephane at magnenat dot net>
    Copyright (C) 2004-2005 Markus Waibel <markus dot waibel at epfl dot ch>
    Copyright (c) 2004-2005 Antoine Beyeler <abeyeler at ab-ware dot com>
    Copyright (C) 2005-2006 Laboratory of Intelligent Systems, EPFL, Lausanne
    Copyright (C) 2006-2008 Laboratory of Robotics Systems, EPFL, Lausanne
    See AUTHORS for details

    This program is free software; the authors of any publication
    arising from research using this software are asked to add the
    following reference:
    Enki - a fast 2D robot simulator
    http://home.gna.org/enki
    Stephane Magnenat <stephane at magnenat dot net>,
    Markus Waibel <markus dot waibel at epfl dot ch>
    Laboratory of Intelligent Systems, EPFL, Lausanne.

    You can redistribute this program and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef __ENKI_IRSENSORWITHCOMM_H
#define __ENKI_IRSENSORWITHCOMM_H

#include <enki/PhysicalEngine.h>
#include <enki/Interaction.h>
#include <enki/interactions/IRSensor.h>
#include <enki/interactions/Radio.h>
#include <enki/Geometry.h>
/*!	\file IRSensor.h
  \brief Header of the generic infrared sensor
*/

namespace Enki
{
  class IRComm;

  typedef RadioMessage<int> IRMessage;
  typedef Radio<int> IRCommRadio;
  // class IRCommRadio : public Radio<int>
  // {
  //   std::vector<int> sources;
  //   public: add_source()
  // }

  struct IRCommEvent
  {
    int rx_value;
    double phase;
    std::vector<int> intensities;
    std::vector<int> payloads;
  };

  std::ostream &operator<<(std::ostream &os, IRCommEvent const &event);

  double responseFunction(double x, double range, double m, double c, double x0);

  class IRComm : public GlobalInteraction
  {
  private:
    IRMessage message();
    static IRCommRadio *radio;
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
    //! Position of the maximum of response (might be negative, inside the robot), second parametere of response function
    const double x0;
    //! Third parameter of response function
    const double c;
    //! Standard deviation of Gaussian noise in the response space
    const double noiseSd;

    double min_intensity;

  public:
    // IRComm(Robot *owner, std::vector<IRSensor *> sensors) : enabled(false), sensors(sensors), tx_value(0) {};
    IRComm(Robot *owner, double range=25, double period=0.1, double aperture=0.644, double m=4200, double x0=0.02, double c=275, double noiseSd = 0.) :
    GlobalInteraction(owner), enabled(false), tx_value(0), range(range),
    time(0), last_sent(-1), period(period), receiver_aperture(aperture),
    m(m), x0(x0), c(c), noiseSd(noiseSd) {
      min_intensity = responseFunction(range, range, m, c, x0);
    }

    void add_sensor(IRSensor * sensor) { sensor->setSearchRange(range); sensors.push_back(sensor); }
    void init(double dt, World* w);
    void finalize(double dt, World* w);
    void step(double dt, World *w);
    void receive_events();
    ~IRComm() { } ;
    void set_enable(bool value) { enabled=value; last_sent=time;}
    void set_tx(int value) { tx_value=value; }
    std::vector<IRCommEvent> get_events() { return events; }

  };
}

#endif

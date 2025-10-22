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

#ifndef __ENKI_IRSENSOR_REALISTIC_H
#define __ENKI_IRSENSOR_REALISTIC_H

#include <enki/interactions/IRSensor.h>

namespace Enki
{
	// TODO: Document
	class IRSensorRealistic : public IRSensor
	{
	public:
		IRSensorRealistic(Robot *owner, Vector pos, double height, double orientation, double range, double m, double x0, double c, double noiseSd = 0., unsigned int rays = 3, double _aperture = 15.0, double k = 6.0, double min_value = 1000.);
		void finalize(double dt, World* w);
	protected:
		void updateRay(size_t i, double dist);
		std::vector<double> ray_weights;
		double ray_weights_sum;
		unsigned int min_value;
		double alpha;
	};

}

#endif

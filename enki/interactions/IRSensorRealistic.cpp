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

#include "IRSensorRealistic.h"

#define W 0.8
#define MIN_VALUE 1000

namespace Enki
{

IRSensorRealistic::IRSensorRealistic(Robot *owner, Vector pos, double height, double orientation, double range, double m, double x0, double c, double noiseSd):
IRSensor(owner, pos, height, orientation, range, m, x0, c, noiseSd)
{

}

void IRSensorRealistic::finalize(double dt, World* w)
{
	if (rayDists[1] > range)
	{
		finalValue = 0;
	}
	else
	{
		double dx1 = rayDists[1] - x0;
		double ws[2];
		double s = 0.0;
		for (size_t i = 0; i < 2; i++) {
			int j = i ? 2 : 0;
			if(rayDists[j] > range)
			{
				ws[i] = 0.0;
			}
			else{
				double z = sqrt(rayDists[j] * rayDists[j] + rayDists[1] * rayDists[1] - 2 * cos(aperture) * rayDists[1] * rayDists[j]);
				//cos(angle) *
				double w = rayDists[j] / z * sin(aperture);
				double dx = rayDists[j] - x0;
				s += w / (2 * W + 1.0) * (0.5 / (dx1 * dx1) + W / (dx * dx));
			}
		}
		if(s == 0.0)
		{
			finalValue = 0;
		}
		else{
			double d = 1.0 + 1.0 / s / (c - x0*x0);
			if(d > 1.0)
			{
				finalValue = m / d;
			}
			else
			{
				d = m;
			}
		}
		finalValue = std::max(0., std::min(m, gaussianRand(finalValue, noiseSd)));
	}
	if(finalValue < MIN_VALUE)
	{
		finalValue = 0.0;
		finalDist = range;
	}
	else{
		finalDist = inverseResponseFunction(finalValue);
	}
}

void IRSensorRealistic::updateRay(size_t i, double dist)
{
	if (dist < rayDists[i])
	{
		rayDists[i] = dist;
	}
}

}

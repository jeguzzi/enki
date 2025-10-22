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


namespace Enki
{

IRSensorRealistic::IRSensorRealistic(Robot *owner, Vector pos, double height, double orientation, double range, double m, double x0, double c, double noiseSd, unsigned int rays, double _aperture, double k, double min_value):
IRSensor(owner, pos, height, orientation, range, m, x0, c, noiseSd, rays, _aperture),
min_value(min_value)
{
	ray_weights_sum = 0.0;
	ray_weights.resize(rays);
	for (size_t i = 0; i < rays; i++) {
		ray_weights[i] = pow(cos(rayAngles[i]), k);
		ray_weights_sum += ray_weights[i];
		// printf("ps %d@%.2f -> %.2f\n", i, rayAngles[i], ray_weights[i]);
	}
	alpha = aperture / ((rays -1) / 2);
}


// void IRSensorRealistic::finalize(double dt, World* w)
// {
// 	// printf("o %.2f \n",  orientation);
// 	if (rayDists[1] > range)
// 	{
// 		// printf("NO \n");
// 		finalValue = 0;
// 	}
// 	else
// 	{
// 		// printf("%.2f , %.2f, %.2f\n", rayDists[0], rayDists[1], rayDists[2]);
// 		double dx1 = rayDists[1] - x0;
// 		double ws[2];
// 		double s = 0.0;
// 		for (size_t i = 0; i < 2; i++) {
// 			int j = i ? 2 : 0;
// 			if(rayDists[j] > range)
// 			{
// 				ws[i] = 0.0;
// 			}
// 			else{
// 				double z = sqrt(rayDists[j] * rayDists[j] + rayDists[1] * rayDists[1] - 2 * cos(aperture) * rayDists[1] * rayDists[j]);
// 				//cos(angle) *
// 				double w = rayDists[j] / z * sin(aperture);
// 				double dx = rayDists[j] - x0;
// 				s += w / (2 * ray_weights[j] + 1.0) * (0.5 / (dx1 * dx1) + ray_weights[j] / (dx * dx));
// 			}
// 		}
// 		if(s == 0.0)
// 		{
// 			finalValue = 0;
// 		}
// 		else{
// 			double d = 1.0 + 1.0 / s / (c - x0*x0);
// 			if(d > 1.0)
// 			{
// 				finalValue = m / d;
// 			}
// 			else
// 			{
// 				d = m;
// 			}
// 		}
// 		finalValue = std::max(0., std::min(m, gaussianRand(finalValue, noiseSd)));
// 	}
// 	if(finalValue < MIN_VALUE)
// 	{
// 		finalValue = 0.0;
// 		finalDist = range;
// 	}
// 	else{
// 		finalDist = inverseResponseFunction(finalValue);
// 	}
// }

void IRSensorRealistic::finalize(double dt, World* w)
{
	// printf("o %.2f \n",  orientation);
	// for (size_t i = 0; i < rayAngles.size(); i++) {
	// 	printf("r%d %.2f \n", i, rayDists[i]);
	// }
	double s = 0.0;
	// z = cos(beta) where beta is the angle between ray and obstacle (on both side of the intersection)
	double zs[2*rayAngles.size()];
	for (size_t i = 0; i < rayAngles.size() - 1; i++) {
		double a = rayDists[i];
		double b = rayDists[i+1];
		if(a > range || b > range)
		{
			zs[2*i + 1] = 0;
			zs[2*i + 2] = 0;
		}
		else{
			// length of the obstacle segment delimited by the two rays
			double _c = sqrt(a * a + b * b - 2 * cos(alpha) * a * b);
			// This would be the mid ray z
			// double z = sin(alpha / 2) * (a + b) / c;
			zs[2*i + 2] = sin(alpha) * a / _c;
			zs[2*i + 1] = sin(alpha) * b / _c;
		}
	}
	zs[0] = 0;
	zs[2*rayAngles.size()-1] = 0;

	// for (size_t i = 0; i < rayAngles.size(); i++) {
	// 	printf("r%d %.2f %.2f \n", i, zs[2*i + 1], zs[2*i + 0]);
	// }

	for (size_t i = 0; i < rayAngles.size(); i++) {
		double dx = rayDists[i] - x0;
		if(dx > 0)
		{
			s += ray_weights[i] * 0.5 / (dx* dx) * std::max(zs[2*i + 1], zs[2*i + 0]);
		}
		else
		{
			s += 1e6;
			break;
		}
	}
	if(s == 0.0)
	{
		finalValue = 0;
	}
	else{
		s /= ray_weights_sum;
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
	if(finalValue < min_value)
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

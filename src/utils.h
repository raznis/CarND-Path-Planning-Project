/*
 * utils.h
 *
 *  Created on: Oct 27, 2017
 *      Author: raz
 */

#ifndef UTILS_H_
#define UTILS_H_
#include <math.h>
using namespace std;

namespace utils
{
	// For converting back and forth between radians and degrees.
	constexpr double pi() { return M_PI; }
	double deg2rad(double x) { return x * pi() / 180; }
	double rad2deg(double x) { return x * 180 / pi(); }
	double mph2ms(double mph){return mph * 0.44704;};
	double distance(double x1, double y1, double x2, double y2)
	{
		return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	};
}

#endif /* UTILS_H_ */

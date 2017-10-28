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
	constexpr double pi();
	double deg2rad(double x);
	double rad2deg(double x);
	double mph2ms(double mph);
	double distance(double x1, double y1, double x2, double y2);
}

#endif /* UTILS_H_ */

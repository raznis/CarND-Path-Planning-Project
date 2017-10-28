/*
 * utils.cpp
 *
 *  Created on: Oct 28, 2017
 *      Author: raz
 */
#include "utils.h"
using namespace std;

namespace utils
{
	constexpr double pi() { return M_PI; }
	double deg2rad(double x) { return x * pi() / 180; }
	double rad2deg(double x) { return x * 180 / pi(); }
	double mph2ms(double mph){return mph * 0.44704;}
	double distance(double x1, double y1, double x2, double y2)
	{
		return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	}

}



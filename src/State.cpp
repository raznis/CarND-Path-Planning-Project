/*
 * State.cpp
 *
 *  Created on: Oct 27, 2017
 *      Author: raz
 */

using namespace std;
#include "State.h"


int State::get_lane(double d)
{
	int lane = int(d / this->map.lane_width);

	if (lane < 0)
		lane = 0;
	else if (lane > this->map.num_lanes - 1)
		lane = this->map.num_lanes-1;

	return lane;
}



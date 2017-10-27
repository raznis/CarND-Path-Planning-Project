/*
 * Map.h
 *
 *  Created on: Oct 27, 2017
 *      Author: raz
 */

#ifndef MAP_H_
#define MAP_H_

#include <sstream>
#include <fstream>

#include <vector>
#include <string>

using namespace std;

class Map {
public:

	Map(double speed_limit, int num_lanes, double lane_width, string waypoints_file);

	/**
	* Destructor
	*/
	virtual ~Map();

	int num_lanes;

	double lane_width; // in meters

	double speed_limit;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// The max s value before wrapping around the track back to 0
	const double max_s = 6945.554;

};



#endif /* MAP_H_ */

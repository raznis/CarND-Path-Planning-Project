/*
 * BehavioralPlanner.h
 *
 *  Created on: Oct 27, 2017
 *      Author: raz
 */

#ifndef BEHAVIORALPLANNER_H_
#define BEHAVIORALPLANNER_H_

#include <iostream>
#include <vector>
#include "Map.h"

using namespace std;



/*
 * This class creates the state and performs high level planning
 */
class BehavioralPlanner{


public:
	State state;
	Map map;

	BehavioralPlanner(Map &_map) {map = _map; state = State(map);};

	virtual ~BehavioralPlanner();

	double distance(double x1, double y1, double x2, double y2);

	int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

	int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

	// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
	vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

	// Transform from Frenet s,d coordinates to Cartesian x,y
	vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
};





#endif /* BEHAVIORALPLANNER_H_ */

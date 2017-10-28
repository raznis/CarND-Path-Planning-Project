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
#include "RoadMap.h"
#include "State.h"
//#include "utils.h"

using namespace std;

enum HighwayFSM
{
	KEEP_LANE,
	LANE_CHANGE_LEFT,
	LANE_CHANGE_RIGHT
};

/*
 * This class creates the state and performs high level planning
 */
class BehavioralPlanner{


public:
	State state;
	RoadMap map;
	HighwayFSM fsmState;	//the current state in the highway finite state machine
	double ref_velocity;
	BehavioralPlanner(RoadMap road_map);
	void updateState(const State state){this->state = state;}
	void generatePlan(vector<double> &next_x_vals, vector<double> &next_y_vals, double end_path_s);
	virtual ~BehavioralPlanner();

};





#endif /* BEHAVIORALPLANNER_H_ */

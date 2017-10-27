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

};





#endif /* BEHAVIORALPLANNER_H_ */

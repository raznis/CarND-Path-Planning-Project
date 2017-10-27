/*
 * State.h
 *
 *  Created on: Oct 27, 2017
 *      Author: raz
 */

#ifndef STATE_H_
#define STATE_H_

using namespace std;

enum HighwayFSM
{
	KEEP_LANE,
	LANE_CHANGE_LEFT,
	LANE_CHANGE_RIGHT
};

class State{

public:
	HighwayFSM fsmState;	//the current state in the highway finite state machine
	Map map;
	double ego_x;
	double ego_y;
	double ego_s;
	double ego_d;
	double ego_yaw;
	double ego_speed;
	int ego_lane;

	State::State(Map &map){fsmState = KEEP_LANE; this->map = map}

	void update(vector<double> EgoData)
	int get_lane(double d);


};


#endif /* STATE_H_ */

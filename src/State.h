/*
 * State.h
 *
 *  Created on: Oct 27, 2017
 *      Author: raz
 */

#ifndef STATE_H_
#define STATE_H_
#include <vector>
using namespace std;



class State{

public:
	double ego_x = 0;
	double ego_y = 0;
	double ego_s = 0;
	double ego_d = 0;
	double ego_yaw = 0;
	double ego_speed = 0;
	vector<vector<double>> sensor_fusion;
	vector<double> prev_x_vals;
	vector<double> prev_y_vals;

	State(){}

	State(double ego_x, double ego_y,double ego_s, double ego_d,
			double ego_yaw, double ego_speed, vector<vector<double>> sensor_fusion,
			vector<double> prev_x_vals, vector<double> prev_y_vals)
	{
		this->ego_x = ego_x;
		this->ego_y = ego_y;
		this->ego_s = ego_s;
		this->ego_d = ego_d;
		this->ego_yaw = ego_yaw;
		this->ego_speed = ego_speed;
		this->sensor_fusion = sensor_fusion;
		this->prev_x_vals = prev_x_vals;
		this->prev_y_vals = prev_y_vals;
	}

};


#endif /* STATE_H_ */

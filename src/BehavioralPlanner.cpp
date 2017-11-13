/*
 * BehavioralPlanner.cpp
 *
 *  Created on: Oct 27, 2017
 *      Author: raz
 */
#include "BehavioralPlanner.h"
#include "spline.h"
#include "utils.h"
using namespace std;


BehavioralPlanner::BehavioralPlanner(RoadMap road_map):map(road_map)
{
	state = State();
	fsmState = KEEP_LANE;
	ref_velocity = 0.0;
};

BehavioralPlanner::~BehavioralPlanner() {}

void BehavioralPlanner::generatePlan(vector<double> &next_x_vals, vector<double> &next_y_vals, double end_path_s)
{
	int lane = map.get_lane(state.ego_d);
	bool my_lane_clear = true;
	bool right_clear = lane < 2;
	bool right_faster = right_clear;
	bool left_clear = lane > 0;
	bool left_faster = left_clear;
	float safe_speed = map.speed_limit - 0.5;
	float safe_speed_right = safe_speed;
	float safe_speed_left = safe_speed;

//	cout<< "my lane: " << lane <<endl;

	int prev_size = state.prev_x_vals.size();

	if(prev_size > 0)
	{
		state.ego_s = end_path_s;
	}

	for (int i = 0; i < state.sensor_fusion.size(); ++i) {
		float check_car_d = state.sensor_fusion[i][6];
		double check_car_vx = state.sensor_fusion[i][3];
		double check_car_vy = state.sensor_fusion[i][4];
		double check_car_speed = sqrt(check_car_vx*check_car_vx+check_car_vy*check_car_vy);
		double check_car_s = state.sensor_fusion[i][5];
		//projection to where our leftover points end (assuming car maintains near 0 lat velocity)
		check_car_s +=((double)prev_size*0.02*check_car_speed);
		//car is in my lane
		if(check_car_d < 2+4*lane+2 && check_car_d > 2+4*lane-2)
		{
			if((check_car_s > state.ego_s) && (check_car_s-state.ego_s) < 70 && check_car_speed < map.speed_limit - 5.0)
			{
				if(safe_speed > check_car_speed)
					safe_speed = check_car_speed;
				my_lane_clear = false;
			}
		}
		if(right_clear){
			if(check_car_d < 2+4*(lane+1)+2 && check_car_d > 2+4*(lane+1)-2){
				//car is in lane to ego's right
				if(std::abs(check_car_s-state.ego_s) < 10)
				{
					right_clear = false;
				}
				else if(check_car_s > state.ego_s && (check_car_s-state.ego_s) < 70){
					if(safe_speed_right > check_car_speed)
						safe_speed_right = check_car_speed;
				}
			}
		}
		if(left_clear){
			if(check_car_d < 2+4*(lane-1)+2 && check_car_d > 2+4*(lane-1)-2){
				//car is in lane to ego's left
				if(std::abs(check_car_s-state.ego_s) < 10)
				{
					left_clear = false;
				}
				else if(check_car_s > state.ego_s && (check_car_s-state.ego_s) < 70){
					if(safe_speed_left > check_car_speed)
						safe_speed_left = check_car_speed;
				}
			}
		}


	}
	right_faster = right_clear && (safe_speed < safe_speed_right - 10.0);
	left_faster = left_clear && (safe_speed < safe_speed_left - 10.0);

	if(right_faster){
		lane = lane + 1;
		if(ref_velocity < safe_speed_right){
			ref_velocity += 0.224;
		}
	}
	else if(left_faster){
		lane = lane - 1;
		if(ref_velocity < safe_speed_left){
			ref_velocity += 0.224;
		}
	}
	else if(!my_lane_clear && ref_velocity > safe_speed){
		ref_velocity -= 0.224;
	}
	else if(ref_velocity < safe_speed){
		ref_velocity += 0.224;
	}



	vector<double> ptsx;
	vector<double> ptsy;

	double ref_x = state.ego_x;
	double ref_y = state.ego_y;
	double ref_yaw = utils::deg2rad(state.ego_yaw);
	if(prev_size < 2)
	{
		double prev_car_x = state.ego_x - cos(state.ego_yaw);
		double prev_car_y = state.ego_y - sin(state.ego_yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(state.ego_x);
		ptsy.push_back(prev_car_y);
		ptsy.push_back(state.ego_y);
	}
	else
	{
		ref_x = state.prev_x_vals[prev_size - 1];
		ref_y = state.prev_y_vals[prev_size - 1];
		double ref_x_prev = state.prev_x_vals[prev_size - 2];
		double ref_y_prev = state.prev_y_vals[prev_size - 2];
		ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	vector<double> next_wp0 = map.getXY(state.ego_s+30.0, (2+4*lane));
	vector<double> next_wp1 = map.getXY(state.ego_s+60.0, (2+4*lane));
	vector<double> next_wp2 = map.getXY(state.ego_s+90.0, (2+4*lane));
	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);
	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	for(int i = 0 ; i < ptsx.size(); i++)
	{
		//shift car reference angle and position to reference point
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;
		ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
		ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
	}

	for(int i = 0 ; i < state.prev_x_vals.size() ; i++)
	{
		next_x_vals.push_back(state.prev_x_vals[i]);
		next_y_vals.push_back(state.prev_y_vals[i]);
	}

	//create a spline
	tk::spline s;
	//set (x,y) points to the spline
	s.set_points(ptsx,ptsy);

	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x*target_x + target_y*target_y);

	double x_add_on = 0;

	for(int i=1; i<=50-state.prev_x_vals.size(); i++)
	{
		double N = target_dist/(0.02*ref_velocity/2.24);
		//creating uniform points on the spline
		double x_point = x_add_on + (target_x/N);
		double y_point = s(x_point);

		x_add_on = x_point;

		//rotate and shift back to global because we are in relative frame
		double x_ref = x_point;
		double y_ref = y_point;

		x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
		y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

		x_point +=ref_x;
		y_point +=ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}
	return;

}








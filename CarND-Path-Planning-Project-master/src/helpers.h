#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <tuple>
#include <set>
#include "Eigen-3.3/Eigen/Dense"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

vector<vector<double>> SE2Transform(const vector<double> &x_R1, const vector<double> &y_R1,
	const double &xO1_R2, const double &yO1_R2, const double &theta1_R2) {
	// Transforming a series of points from 1 to 2:
	// xO1_R2, yO1_R2 are the x,y coords of O1 represented in (X2,Y2)
	// theta1_R2 is the rotation angle of (X1,Y1) relative to (X2,Y2)
	vector<double> x_R2, y_R2;
	int length = x_R1.size();
	double x1, y1;
	for (int i = 0; i < length; i++) {
		x1 = x_R1[i];
		y1 = y_R1[i];
		x_R2.push_back(cos(theta1_R2) * x1 - sin(theta1_R2) * y1 + xO1_R2);
		y_R2.push_back(sin(theta1_R2) * x1 + cos(theta1_R2) * y1 + yO1_R2);
	}
	return { x_R2, y_R2 };
}
vector<double> SE2Transform(const double &x_R1, const double &y_R1,
	const double &xO1_R2, const double &yO1_R2, const double &theta1_R2) {
	// Transforming one point from 1 to 2:
	// xO1_R2, yO1_R2 are the x,y coords of O1 represented in (X2,Y2)
	// theta1_R2 is the rotation angle of (X1,Y1) relative to (X2,Y2).
	// R1|2 = [[x1|2,y1|2],[O1|2;1]];
	// v|2 = R1|0 * v|1
	double x_R2, y_R2;
	x_R2 = (cos(theta1_R2) * x_R1 - sin(theta1_R2) * y_R1 + xO1_R2);
	y_R2 = (sin(theta1_R2) * x_R1 + cos(theta1_R2) * y_R1 + yO1_R2);
	return { x_R2, y_R2 };
}

vector<vector<double>> GlobalToCarTransform(const vector<double> &x_g, const vector<double> &y_g,
	const double &xCar_g, const double &yCar_g, const double &heading) {
	// Know global coordinates x_g, y_g, want to get car coordinates x_car, y_car
	// xCar_g, yCar_g, heading are place and angle of car in global coordinates
	double xOGlobal_car = -(xCar_g * cos(heading) + yCar_g * sin(heading));
	double yOGlobal_car = -(yCar_g * cos(heading) - xCar_g * sin(heading));
	double theta = -heading;
	return SE2Transform(x_g, y_g, xOGlobal_car, yOGlobal_car, theta);
}
vector<double> GlobalToCarTransform(const double &x_g, const double &y_g,
	const double &xCar_g, const double &yCar_g, const double &heading) {
	// Know global coordinates x_g, y_g, want to get car coordinates x_car, y_car
	// xCar_g, yCar_g, heading are place and angle of car in global coordinates
	double xOGlobal_car = -(xCar_g * cos(heading) + yCar_g * sin(heading));
	double yOGlobal_car = -(yCar_g * cos(heading) - xCar_g * sin(heading));
	double theta = -heading;
	return SE2Transform(x_g, y_g, xOGlobal_car, yOGlobal_car, theta);
}

void printVector(vector<double> v) {
	for (int i = 0; i < v.size(); i++) {
		std::cout << v[i] << ' ';
	}
	std::cout << ' ' << std::endl;
}
void printVector(vector<int> v) {
	for (int i = 0; i < v.size(); i++) {
		std::cout << v[i] << ' ';
	}
	std::cout << ' ' << std::endl;
}


inline int getLaneId(double d, double yellow_line_d, double lane_width) {

	if ((d - yellow_line_d) < lane_width) { return 0; }
	else if ((d - yellow_line_d) < lane_width * 2) { return 1; }
	else { return 2; }

}


void setACCSpeedAndAcceleration(double & ref_speed, double & ref_accel,
	const double set_speed, const double distance_to_predecesor, const double T) {

	double speed_increment = ref_accel * (1.0*T);
	double k_accel;
	if (ref_speed > set_speed + speed_increment) {
		ref_speed -= speed_increment; // using -5m/s^2 accel
		k_accel = -1.0;
		if (distance_to_predecesor < ref_speed * T) {
			k_accel -= 2.0* (1.2 - (check_car_s - car_s) / (ref_speed * T));
		}
	}
	else if (ref_speed < set_speed - speed_increment) {
		ref_speed += speed_increment; // using -5m/s^2 accel
		if (ref_speed < 0.5 * set_speed) { // ref_speed very low, need to accelerate fast
			k_accel = 2.0;
		}
		else {
			k_accel = 1.0;
		}
	}
	else {
		ref_speed = set_speed;
		k_accel = +0.0;
		if (distance_to_predecesor < ref_speed * T) {
			k_accel -= 2.0 * (1.0 - std::max(1.0, distance_to_predecesor / (ref_speed * T)) );
		}
	}
	ref_accel *= k_accel;   // update ref_accel for generating future waypoints.
}



bool checkVehicleInSegment(int lane_id, double yellow_line_d, double lane_width,
	double dT, double segment_start, double check_car_s, double check_speed, vector<vector<double>>sensor_fusion) {
	bool is_ocupied = false;
	for (int i = 0; i < sensor_fusion.size(); i++) {
		double check_d = sensor_fusion[i][6];
		if ((check_d > yellow_line_d + (double)lane_id*lane_width) && (check_d < yellow_line_d + (double)lane_id*lane_width + lane_width)) {
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double v = sqrt(pow(vx, 2) + pow(vy, 2));
			double prev_check_car_s = sensor_fusion[i][5];
			double s = prev_check_car_s + dT * v;

			if (((s >= segment_start) && (s < check_car_s))) {
				is_ocupied = true;
				return is_ocupied;
			}
		}
	}
	return is_ocupied;
}

bool findPredecessorInSegment(int lane_id, double yellow_line_d, double lane_width,
	double dT, double segment_start, double & check_car_s, double & check_speed, vector<vector<double>>sensor_fusion) {
	bool is_ocupied = false;
	for (int i = 0; i < sensor_fusion.size(); i++) {
		double check_d = sensor_fusion[i][6];
		if ((check_d > yellow_line_d + (double)lane_id*lane_width) && (check_d < yellow_line_d + (double)lane_id*lane_width + lane_width)) {
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double v = sqrt(pow(vx, 2) + pow(vy, 2));
			double prev_check_car_s = sensor_fusion[i][5];
			double s = prev_check_car_s + dT * v;
			
			if (((s >= segment_start) && (s < check_car_s))) {
				is_ocupied = true;
				if (s < check_car_s) {
					check_car_s = s;
					check_speed = v;
				}
			}
		}
	}
	return is_ocupied;
}

//
//int checkLaneEmpty(int lane_id, vector<vector<double>> lane_id_s_list, 
//	vector<vector<double>> sensor_fusion, double max_map_s, 
//	double yellow_line_d, double lane_width) {
//	int output = 0;
//
//	for (int j = 0; j < lane_id_s_list.size(); j++) {
//		double s_start = lane_id_s_list[j][0];
//		double s_end = lane_id_s_list[j][1];
//
//		for (int i = 0; i < sensor_fusion.size(); i++) {
//			if (lane_id == getLaneId(sensor_fusion[i][6], yellow_line_d, lane_width)) {
//				if ((sensor_fusion[i][5] >= s_start) && (sensor_fusion[i][5] < s_end)) {
//					output += 1;
//					return output;
//				}
//				if ((sensor_fusion[i][5] + max_map_s >= s_start) && (sensor_fusion[i][5] + max_map_s < s_end)) {
//					output += 1;
//					return output;
//				}
//			}
//		}
//	}
//	return output;
//}

//inline double calculatePolynomial(vector<double> c, double t) {
//	double y = 0.0;
//	for (int i = 0; i < c.size(); i++) {
//		y += c[i] * pow(t, i);
//	}
//	return y;
//}
//vector<double> JMT(vector<double> &start, vector<double> &end, double T_start, double T_end) {
//	/**
//	 * Calculate the Jerk Minimizing Trajectory that connects the initial state
//	 * to the final state in time T.
//	 *
//	 * @param start - the vehicles start location given as a length three array
//	 *   corresponding to initial values of [s, s_dot, s_double_dot]
//	 * @param end - the desired end state for vehicle. Like "start" this is a
//	 *   length three array.
//	 * @param T - The duration, in seconds, over which this maneuver should occur.
//	 *
//	 * @output an array of length 6, each value corresponding to a coefficent in
//	 *   the polynomial:
//	 *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
//	 *
//	 * EXAMPLE
//	 *   > JMT([0, 10, 0], [10, 10, 0], 1)
//	 *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
//	 */
//	double T = T_end - T_start;
//	// parameters a0~a5: time span [0,T]
//	double a0 = start[0];
//	double a1 = start[1];
//	double a2 = start[2] / 2.0;
//	MatrixXd A(3, 3);
//	A << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
//		3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4),
//		6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3);
//	VectorXd b(3);
//	b << end[0] - (start[0] + start[1] * T + 0.5*start[2] * T*T),
//		end[1] - (start[1] + start[2] * T),
//		end[2] - start[2];
//
//	VectorXd a345 = A.colPivHouseholderQr().solve(b);
//
//	double a3 = a345[0];
//	double a4 = a345[1];
//	double a5 = a345[2];
//
//	// parameters c0~c5: time span [T_start, T_end]
//	double T_start2 = T_start * T_start;
//	double T_start3 = T_start * T_start2;
//	double T_start4 = T_start * T_start3;
//	double T_start5 = T_start * T_start4;
//
//	double c0 = a0 - a1 * T_start + a2 * T_start2 - a3 * T_start3 + a4 * T_start4 - a5 * T_start5;
//	double c1 = a1 - a2 * T_start * 2 + a3 * 3 * T_start2 - a4 * 4 * T_start3 + a5 * 5 * T_start4;
//	double c2 = a2 - a3 * 3 * T_start + a4 * 6 * T_start2 - a5 * 10 * T_start3;
//	double c3 = a3 - a4 * 4 * T_start + a5 * 10 * T_start2;
//	double c4 = a4 - a5 * 5 * T_start;
//	double c5 = a5;
//
//	return { c0,c1,c2,c3,c4,c5 };
//}


//
//std::tuple<vector<int>, vector<vector<double>> > getRegionToTravel(double end_previous_path_s, double end_previous_path_d, 
//	double car_s, double car_d, double set_speed, int N, int previous_length, double max_map_s, 
//	double yellow_line_d, double lane_width, double dT) {
//	// Generate the list of lane_id(s) and list of road segments in s (Frenet) coordinate
//	// in T=1second horizon, at most one lane change is planned. 
//	// If car_d == end_path_d, then there is not lane change.
//	// Otherwise, there is lane change.
//	vector<int> planned_lane_id_list;
//	vector<vector<double>> planned_lane_s_list;
//	if (true) { // && (getLaneId(car_d, yellow_line_d, lane_width) == getLaneId(end_path_d, yellow_line_d, lane_width))) { 
//		// case with no lane change
//		double end_path_s = end_previous_path_s + set_speed * (N - previous_length) * dT;
//		if (end_path_s <= max_map_s) { // no looping the cycle happening.
//			planned_lane_id_list.push_back(getLaneId(car_d, yellow_line_d, lane_width));
//			planned_lane_s_list.push_back({ car_s, end_path_s });
//		}
//		else if (car_s <= end_previous_path_s) { // loop the cycle when extending the plan
//			planned_lane_id_list.push_back(getLaneId(car_d, yellow_line_d, lane_width));
//			planned_lane_id_list.push_back(getLaneId(car_d, yellow_line_d, lane_width));
//			planned_lane_s_list.push_back({ car_s, max_map_s });
//			planned_lane_s_list.push_back({ 0, fmod(end_path_s, max_map_s) });
//		}
//		else { // loop the cycle in the previous planned path
//			planned_lane_id_list.push_back(getLaneId(car_d, yellow_line_d, lane_width));
//			planned_lane_id_list.push_back(getLaneId(car_d, yellow_line_d, lane_width));
//			planned_lane_s_list.push_back({ car_s, max_map_s });
//			planned_lane_s_list.push_back({ 0, fmod(end_path_s, max_map_s) });
//		}
//	}
//	else {// it there is lane change
//	}
//	return std::make_tuple(planned_lane_id_list, planned_lane_s_list);
//}

//
//std::tuple<int, vector<double>> planBehaviorAndTraj(vector<int> planned_lane_id_list,
//	vector<vector<double>> planned_lane_s_list, vector<vector<double>> sensor_fusion, 
//	double T, double dT, int previous_length, 
//	vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d,
//	double set_speed, double max_map_s,
//	double car_s, double car_d, double car_speed, 
//	double yellow_line_d, double lane_width) {
//	// mode: 
//	// [0]: keep lane, re-use previous traj and track set_speed. {T_planned,T_end,s_f,ds_f,dds_f}. 
//	// [1]: planned keep lane, generate new trajectory. Output minimum jerk traj {T_end,s_f,ds_f,dds_f},T_planned=0.
//	// [2]: lane change left, re-use previous traj. Output minimum jerk traj {T_planned,T_end,s_f,ds_f,dds_f,d_f,d(d)_f,dd(d)_f}
//	// [3]: planned lane change left, generate new traj. Output minimum jerk traj {T_end,s_f,ds_f,dds_f,d_f,d(d)_f,dd(d)_f},T_planned=0.
//	// [4]: lane change right, re-use previous traj. Output minimum jerk traj {T_planned,T_end,s_f,ds_f,dds_f,d_f,d(d)_f,dd(d)_f}
//	// [5]: planned lane change right, generate new traj. Output minimum jerk traj {T_end,s_f,ds_f,dds_f,d_f,d(d)_f,dd(d)_f},T_planned=0.
//	// [6]: no planned trajectory. 
//
//	// Calculate behavior_mode:
//	// Plan new trajectory if there is other car in planned_lane_s_list
//	// bool b_lane_change_planned = (std::set<int>(planned_lane_id_list.begin(), planned_lane_id_list.end()).size() > 1);
//	int behavior_mode;
//	int lane_id2;
//	// max_map_s = 6945.554;
//	if (false && previous_length >= 3) {
//		if (*planned_lane_id_list.begin() == *(planned_lane_id_list.end() - 1)) { // mode 0 or 1.
//			// check if there is vehicle inside (d,s) range
//			behavior_mode = 0; // +checkLaneEmpty(*planned_lane_id_list.begin(), planned_lane_s_list, sensor_fusion);
//			lane_id2 = *planned_lane_id_list.begin();
//		}
//		else if (*planned_lane_id_list.begin() > *(planned_lane_id_list.end() - 1)) { // mode 2 or 3
//			int lane_id1 = *planned_lane_id_list.begin();
//			lane_id2 = *(planned_lane_id_list.end() - 1);
//			vector<vector<double>> lane_id1_s_list, lane_id2_s_list;
//			for (int i = 0; i < planned_lane_id_list.size(); i++) {
//				if (planned_lane_id_list[i] == lane_id1) {
//					lane_id1_s_list.push_back(planned_lane_s_list[i]);
//				}
//				else {
//					lane_id2_s_list.push_back(planned_lane_s_list[i]);
//				}
//			}
//			behavior_mode = 2 + std::max(checkLaneEmpty(lane_id1, lane_id1_s_list, sensor_fusion, max_map_s, yellow_line_d, lane_width),
//				checkLaneEmpty(lane_id2, lane_id2_s_list, sensor_fusion, max_map_s, yellow_line_d, lane_width));
//		}
//		else { // mode 4 or 5
//			int lane_id1 = *planned_lane_id_list.begin();
//			lane_id2 = *(planned_lane_id_list.end() - 1);
//			vector<vector<double>> lane_id1_s_list, lane_id2_s_list;
//			for (int i = 0; i < planned_lane_id_list.size(); i++) {
//				if (planned_lane_id_list[i] == lane_id1) {
//					lane_id1_s_list.push_back(planned_lane_s_list[i]);
//				}
//				else {
//					lane_id2_s_list.push_back(planned_lane_s_list[i]);
//				}
//			}
//			behavior_mode = 4 + std::max(checkLaneEmpty(lane_id1, lane_id1_s_list, sensor_fusion, max_map_s, yellow_line_d, lane_width),
//				checkLaneEmpty(lane_id2, lane_id2_s_list, sensor_fusion, max_map_s, yellow_line_d, lane_width));
//		}
//	}
//	else {
//		behavior_mode = 6;
//	}
//
//
//	// Generate totally new trajectory or continuing trajectory based on mode:
//	vector<double> behavior_parameters;
//	if (behavior_mode == 0 || behavior_mode == 2 || behavior_mode == 4) {
//		double T_planned = previous_length * dT;
//		double T_end = std::max(T, T_planned+T*2.0);
//		double s_start = end_path_s;
//		double d_s_start = sqrt(pow((previous_path_x[previous_length-1]- previous_path_x[previous_length - 2])/dT,2) + 
//			pow((previous_path_y[previous_length - 1] - previous_path_y[previous_length - 2]) / dT, 2));
//		double d_s_second_last = sqrt(pow((previous_path_x[previous_length - 2] - previous_path_x[previous_length - 3]) / dT, 2) +
//			pow((previous_path_y[previous_length - 2] - previous_path_y[previous_length - 3]) / dT, 2));
//		double dd_s_start = (d_s_start - d_s_second_last) / dT;
//		double s_end = end_path_s + (T_end - T_planned) * set_speed;
//		double d_s_end = set_speed;
//		double dd_s_end = 0.0;
//		std::cout << "original s_end is " << s_end << std::endl;
//		// look for preceding vehicle and track
//		for (int i = 0; i < sensor_fusion.size(); i++) {
//			if (lane_id2 == getLaneId(sensor_fusion[i][6], yellow_line_d, lane_width)) {
//				if ((sensor_fusion[i][5] >= s_start) && (sensor_fusion[i][5] < s_end)) {
//					s_end = sensor_fusion[i][5];
//					d_s_end = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));
//				}
//				if ((sensor_fusion[i][5] + max_map_s >= s_start) && (sensor_fusion[i][5] + max_map_s < s_end)) {
//					s_end = sensor_fusion[i][5] + max_map_s;
//					d_s_end = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));
//				}
//			}
//		}
//		std::cout << "modified s_end is " << s_end << std::endl;
//		// Polynomial coefficients
//		vector<double> start{ s_start,d_s_start,dd_s_start };
//		vector<double> end{ s_end,d_s_end,dd_s_end };
//
//		behavior_parameters = JMT(start, end, T_planned, T_end);
//
//		std::cout << "T_planned is " << T_planned << std::endl;
//		printVector(behavior_parameters);
//	}
//	else { // plan a new trajectory following lane
//
//		double T_planned = 0;
//		double T_end = T;
//		double s_start = car_s;
//		double d_s_start = car_speed;
//		double dd_s_start = 0.0;
//		double s_end = car_s + (T_end - T_planned) * set_speed;
//		double d_s_end = set_speed;
//		double dd_s_end = 0.0;
//		lane_id2 = getLaneId(car_d, yellow_line_d, lane_width);
//		// look for preceding vehicle and track
//		for (int i = 0; i < sensor_fusion.size(); i++) {
//			if (lane_id2 == getLaneId(sensor_fusion[i][6], yellow_line_d, lane_width)) {
//				if ((sensor_fusion[i][5] >= s_start) && (sensor_fusion[i][5] < s_end)) {
//					s_end = sensor_fusion[i][5];
//					d_s_end = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));
//				}
//				if ((sensor_fusion[i][5] + max_map_s >= s_start) && (sensor_fusion[i][5] + max_map_s < s_end)) {
//					s_end = sensor_fusion[i][5] + max_map_s;
//					d_s_end = sqrt(pow(sensor_fusion[i][3], 2) + pow(sensor_fusion[i][4], 2));
//				}
//			}
//		}
//		// Polynomial coefficients
//		vector<double> start{ s_start,d_s_start,dd_s_start };
//		vector<double> end{ s_end,d_s_end,dd_s_end };
//
//		behavior_parameters = JMT(start, end, T_planned, T_end);
//	}
//
//	return std::make_tuple(behavior_mode, behavior_parameters);
//}
//

#endif  // HELPERS_H
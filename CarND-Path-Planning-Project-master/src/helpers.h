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


bool checkVehicleInSegment(int lane_id, double yellow_line_d, double lane_width,
	double dT, double segment_start, double check_car_s, vector<vector<double>>sensor_fusion) {
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


bool setTargetLane(int & lane_id, const double set_speed, const double car_s, const double max_speed,
	const double yellow_line_d, const double lane_width,
	const double T, const double dT, vector<vector<double>> sensor_fusion) {
	// Called when want to make lane change.
	// lane_id will be updated if target lane is not the current lane_id.
	if (lane_id != 0) {
		// If the planned front region is not occupied in the new lane
		// want to change to (lane_id-1)
		bool left_lane_is_ocupied = checkVehicleInSegment(lane_id - 1,
			yellow_line_d, lane_width, dT, car_s - max_speed * T,
			car_s + max_speed * T * 1.5, sensor_fusion);
		if (!left_lane_is_ocupied) {
			lane_id -= 1;
			return true;
		}
	}
	else {
		bool right_lane_is_ocupied = checkVehicleInSegment(lane_id + 1,
			yellow_line_d, lane_width, dT, car_s - max_speed * T,
			car_s + max_speed * T * 1.5, sensor_fusion);
		if (!right_lane_is_ocupied) {
			lane_id += 1;
			return true;
		}
	}
	return false;
}


void setACCSpeedAndAcceleration(double & ref_speed, double & ref_accel,
	double & set_speed, const double distance_to_predecesor, const double T) {
	// speed_increment should be the speed difference from the timestamp when previous_path is received,
	// for the speed indexed at previous_length. 
	// Then the controller needs to do calculation, and send back to the simulator. 
	// This approximately takes 8~10 steps. 
	// With 2~3m/s^2 acceleration, speed_increment is set to 0.224.
	// This is not going to have huge acceleration.
	double speed_increment = ref_accel * 0.02 * 2.0;  // Use 2*ref_accel to have rapid response.  
	double k_accel;
	if (ref_speed > set_speed + speed_increment) { // Speed higher than predecessor.
		ref_speed -= speed_increment; // using -5m/s^2 accel
		k_accel = -2.0;
		if (distance_to_predecesor < ref_speed * T) {
			set_speed = set_speed * 0.6;  // If distance is close, decrease speed a little. 
		}
	}
	else if (ref_speed < set_speed - speed_increment) {
		if (distance_to_predecesor < ref_speed * T) {
			k_accel = +0.0;   // Speed lower than predecessor, distance quick short, just keep speed. 
		}
		else {  // Speed lower than predecessor, distance quick long, increase speed.
			ref_speed += speed_increment; // using -5m/s^2 accel
			if (ref_speed < 0.5 * set_speed) { // ref_speed very low, need to accelerate fast
				k_accel = 1.5;
			}
			else {
				k_accel = 1.0;
			}
		}
	}
	else {   // Speed close to predecessor.
		ref_speed = set_speed;
		k_accel = +0.0;
		if (distance_to_predecesor < ref_speed * T) {
			k_accel = -2.0; 
			ref_speed -= speed_increment;
			set_speed = set_speed * 0.6;  // If distance is close, decrease speed a little. 
		}
	}
	
	ref_accel *= k_accel;   // update ref_accel for generating future waypoints.
}


#endif  // HELPERS_H
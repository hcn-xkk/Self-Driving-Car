#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
const double Mph2Mps = 0.44704;

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;
	double lane_width = 4.0;
	double yellow_line_d = 0.0;


	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		std::istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
		&map_waypoints_dx, &map_waypoints_dy, &max_s, &yellow_line_d, &lane_width]
		(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
			uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side 
					//   of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					/** 
					------------------------------------------------------------------------
					Generate a path made up of (x,y) points that the car will visit 
					sequentially every .02 seconds
					------------------------------------------------------------------------
					*/
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					double dT = 0.02;   // delta for the sent out trajectories
					double T = 1.0;     // time span of the sent trajectory
					double set_speed = 45.0 * Mph2Mps;
					double lane_change_speed = set_speed * 0.85; 
					double max_speed = 50.0 * Mph2Mps;     // [m/s] max travel speed 50mph
					// ref_speed, ref_accel are used to generate new waypoints.
					// ref_accel can be plus or minus.
					double ref_speed = std::max(0.0, car_speed * Mph2Mps);
					double ref_accel = 0.8;


					// - Find current lane_id:
					int lane_id = getLaneId(car_d, yellow_line_d, lane_width);
					
					// - Find length of previous path:
					int previous_length = previous_path_x.size();
					

					// - Find whether there is preceding vehicle, set set_speed, accel/decel:
					bool lane_is_ocupied = false;
					double check_car_s = car_s + set_speed * T*2.0;
					int old_lane_id = lane_id;
					// Check if segment has other preceding vehicle and update check_car_s, check_speed.
					lane_is_ocupied = findPredecessorInSegment(lane_id, 
						yellow_line_d, lane_width, dT, car_s, check_car_s, set_speed, sensor_fusion);
					if (lane_is_ocupied) { // Re-use a shorter previous path
						if (previous_length >=4) {
							previous_length = (int)(previous_length *0.5);
						}
					}

					
					// Decide change lane:
					bool make_lane_change = false;
					if (lane_is_ocupied && set_speed < lane_change_speed) { //
						make_lane_change = setTargetLane(lane_id, set_speed, car_s, max_speed,
							yellow_line_d, lane_width, T, dT, sensor_fusion);
						if (make_lane_change) {
							ref_accel *= 1; // If make a lane change, decrease longitudinal acceleration.
						}
					}

					if (previous_length >= 2) {
						double x1 = previous_path_x[previous_length - 1];
						double x2 = previous_path_x[previous_length - 2];
						double y1 = previous_path_y[previous_length - 1];
						double y2 = previous_path_y[previous_length - 2];

						ref_speed = sqrt(((x1-x2)/dT)*((x1 - x2) / dT) + ((y1 - y2) / dT)*((y1 - y2) / dT));
						std::cout << "ref_speed " << ref_speed << std::endl;
						std::cout << "set_speed " << set_speed << std::endl;

						//ref_speed = set_speed;
					}


					// Set acceleration / deceleration for generating future waypoints.
					double distance_to_predecesor = check_car_s - car_s;
					setACCSpeedAndAcceleration(ref_speed, ref_accel, set_speed, distance_to_predecesor, T);
					std::cout << "ref_speed 2 " << ref_speed << std::endl;
					std::cout << "set_speed 2 " << set_speed << std::endl;
					std::cout << "ref_accel 2 " << ref_accel << std::endl;

					// - Create x and y waypoints:
					vector<double> new_car_x_waypoints;
					vector<double> new_car_y_waypoints;

					// Push the previous_path_x,previous_path_y or current states into waypoints.
					// This is to have frame-to-frame consistency.
					double ref_yaw, ref_x, ref_y;
					if (previous_length >= 2) {
						// Reuse the previous path in this step
						for (int i = 0; i < previous_length; i++) {
							next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);
						}
						// Constraint the heading using the last two points in the previous path
						ref_y = previous_path_y[previous_length - 1];
						double ref_y_prev = previous_path_y[previous_length - 2];
						ref_x = previous_path_x[previous_length - 1];
						double ref_x_prev = previous_path_x[previous_length - 2];
						ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
						
						new_car_x_waypoints.push_back(ref_x_prev);
						new_car_x_waypoints.push_back(ref_x);
						new_car_y_waypoints.push_back(ref_y_prev);
						new_car_y_waypoints.push_back(ref_y);
					}
					else {
						// Going one step backwards to constraint the heading
						ref_x = car_x;
						ref_y = car_y;
						ref_yaw = car_yaw;

						new_car_x_waypoints.push_back(car_x - 1.0 * cos(car_yaw));
						new_car_y_waypoints.push_back(car_y - 1.0 * sin(car_yaw));
						new_car_x_waypoints.push_back(car_x);
						new_car_y_waypoints.push_back(car_y);
					}

					// Push the future waypoints
					double dist_inc = max_speed * T * 1.2;
					vector<double> farthest_sd = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
					for (int i = 1; i <= 3; i++) {
						double new_car_s;
						new_car_s = farthest_sd[0] + dist_inc * ((double)i+0.0);
						vector<double> new_car_xy;
						if (!make_lane_change) {
							new_car_xy = getXY(new_car_s,
								lane_width / 2.0 + (double)lane_id*lane_width,
								map_waypoints_s, map_waypoints_x, map_waypoints_y);
						}
						else {
							new_car_xy = getXY(new_car_s,
								lane_width / 2.0 + ((double)lane_id*0.8 + (double)old_lane_id*0.2)*lane_width,
								map_waypoints_s, map_waypoints_x, map_waypoints_y);
						}
						new_car_x_waypoints.push_back(new_car_xy[0]);
						new_car_y_waypoints.push_back(new_car_xy[1]);
					}


					// - Interpolate waypoints to generate reference trace:
					// Transform into car coordinates. 
					auto new_car_carxy_waypoints = GlobalToCarTransform(new_car_x_waypoints,
						new_car_y_waypoints, ref_x, ref_y, ref_yaw);
					
					// Create x and y interpolated reference points:
					tk::spline spline_xy_car;   // This is in car coordinates align with heading
					spline_xy_car.set_points(new_car_carxy_waypoints[0], new_car_carxy_waypoints[1]);
					double new_x_car, new_y_car;
					vector<double> new_xy_global;
					double delta_x_car = std::max(10.0,ref_speed) * dT ;   // Assuming car_yaw does not change much in one horizon
					auto starting_xy_car = GlobalToCarTransform(ref_x, ref_y, ref_x, ref_y, ref_yaw);
					double x0_car = starting_xy_car[0];
					vector<double> help_debug_x, help_debug_y, help_debug_v;
					for (int i = 1; i <= T / dT - next_x_vals.size(); i++) {
						// Doing interpolation
						double theta = atan2(spline_xy_car(x0_car + 0.01) - spline_xy_car(x0_car),
							0.01);
						new_x_car = delta_x_car*cos(theta) + x0_car;
						new_y_car = spline_xy_car(new_x_car);

						help_debug_x.push_back(new_x_car);
						help_debug_y.push_back(new_y_car);
						help_debug_v.push_back(ref_speed);


						// Transform back to global coordinates
						new_xy_global = SE2Transform(new_x_car, new_y_car, ref_x, ref_y, ref_yaw);
						next_x_vals.push_back(new_xy_global[0]);
						next_y_vals.push_back(new_xy_global[1]);
						//if (fabs(ref_speed - set_speed) > fabs(ref_accel) * dT) {
						//	ref_speed += 0.224;// ref_accel * dT;
						//}
						//if (ref_accel < 0) {
						//	if ((ref_speed - set_speed) > fabs(ref_accel * dT)) {
						//		ref_speed += ref_accel * dT; //0.224;// ref_accel * dT;
						//	}
						//	else {
						//		ref_speed = set_speed;
						//	}
						//}
						//else if (ref_accel > 0) {
						//	if ((set_speed - ref_speed) > fabs(ref_accel * dT)) {
						//		ref_speed += ref_accel * dT; //  0.224;// ref_accel * dT;
						//	}
						//	else {
						//		ref_speed = set_speed;
						//	}
						//}
						if (fabs(ref_accel) > 1e-3) {
							if (fabs(ref_speed - set_speed) > fabs(ref_accel * dT)) {
								ref_speed += ref_accel * dT; //0.224;// ref_accel * dT;
							}
							else {
								ref_speed = set_speed;
							}
						}
						else {
							ref_speed = set_speed;
						}
						delta_x_car = ref_speed * dT;
						x0_car = new_x_car;
						

					}
					if (true) {
						std::cout << "Previous length " << previous_length << std::endl;
						printVector(next_x_vals);
						printVector(next_y_vals);
						printVector(help_debug_x);
						printVector(help_debug_y);
						printVector(help_debug_v);
						
					}

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}  // end "telemetry" if
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}  // end websocket if
	}); // end h.onMessage

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
		char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}
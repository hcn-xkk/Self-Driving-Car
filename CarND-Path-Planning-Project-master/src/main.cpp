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
#define PI 3.14159265

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

	std::cout << "here!" << std::endl;
	auto output = SE2Transform({ 0.0,2.0 }, { 0.0,3.0 }, 1.0, 2.0, 0.25*PI);
	printVector(output[0]);


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

					vector<double> next_x_vals;
					vector<double> next_y_vals;
					std::cout << " ----------------------- " << std::endl;
					/**
					 * TODO: define a path made up of (x,y) points that the car will visit
					 *   sequentially every .02 seconds
					 */
					 // Try 3 : The last path needs to be smoothened. 

					 // - Use Frenet coord to select a few waypoints,
					 // then use assume constant speed along lane
					double dT = 0.02;   // delta for the sent out trajectories
					double T = 1.0;     // Time span of the sent trajectory
					double set_speed = 48.0 * 0.44;     // [m/s] travel with 50Mph

					// - Find current lane_id:
					// lane width 4, double yellow lane d=0
					int lane_id;
					if (car_d < 4) { lane_id = 0; }
					else if (car_d < 8) { lane_id = 1; }
					else { lane_id = 2; }

					int previous_length = previous_path_x.size();
					// Region to check: [car_s, car_s + set_speed * T]
					vector<int> planned_lane_id_list;
					vector<vector<double>> planned_lane_s_list;
					tie(planned_lane_id_list, planned_lane_s_list) = getRegionToTravel(end_path_s, end_path_d,
						car_s, car_d, set_speed, (int)(T / dT), previous_length, max_s, yellow_line_d, lane_width);
					std::cout << " planned_lane_id_list " << std::endl;
					printVector(planned_lane_id_list);
					std::cout << " planned_lane_s_list " << std::endl;
					printVector(planned_lane_s_list[0]);
					int lane_is_ocupied = checkLaneEmpty(planned_lane_id_list[0], planned_lane_s_list, sensor_fusion, max_s, yellow_line_d, lane_width);
					std::cout << " lane_is_ocupied " << lane_is_ocupied << std::endl;

					// - Create x and y waypoints:
					double new_car_s;
					vector<double> new_car_x_waypoints;
					vector<double> new_car_y_waypoints;
					vector<double> new_car_t_waypoints;

					// Push the previous_path_x,previous_path_y into waypoints:
					// Including previous planned paths will help consistency.
					
					double ref_yaw;
					double ref_y;
					double ref_x;
					
					if ((lane_is_ocupied==0) && previous_length >= 2) {
						std::cout << "Get to the if" << std::endl;
						ref_y = previous_path_y[previous_length - 1];
						double ref_y_prev = previous_path_y[previous_length - 2];
						ref_x = previous_path_x[previous_length - 1];
						double ref_x_prev = previous_path_x[previous_length - 2];
						std::cout << ref_x << ' ' << ref_x_prev << std::endl;

						ref_yaw = std::atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
						new_car_x_waypoints.push_back(ref_x_prev);
						new_car_x_waypoints.push_back(ref_x);
						new_car_y_waypoints.push_back(ref_y_prev);
						new_car_y_waypoints.push_back(ref_y);

						for (int i = 0; i < previous_length; i++) {
							next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);
						}

					}
					else {
						std::cout << "Get to the else" << std::endl;
						ref_yaw = car_yaw;
						// Going one step backwards
						if (car_speed > 0) {
							new_car_x_waypoints.push_back(car_x - car_speed * dT * cos(car_yaw));
							new_car_y_waypoints.push_back(car_y - car_speed * dT * sin(car_yaw));
							// Push the current point
							new_car_x_waypoints.push_back(car_x);
							new_car_y_waypoints.push_back(car_y);
						}
						else {
							new_car_x_waypoints.push_back(car_x - 1 * dT * cos(car_yaw));
							new_car_y_waypoints.push_back(car_y - 1 * dT * sin(car_yaw));
							// Push the current point
							new_car_x_waypoints.push_back(car_x);
							new_car_y_waypoints.push_back(car_y);
						}
						ref_x = car_x;
						ref_y = car_y;


					}

					// Push the future points
					double dist_inc = set_speed * T / 3.0;
					for (int i = 1; i <= 3; i++) {
						//
						vector<double> farthest_sd = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
						new_car_s = farthest_sd[0] + dist_inc * i;
						vector<double> new_car_xy = getXY(new_car_s, 2.0 + (double)lane_id*4.0,
							map_waypoints_s, map_waypoints_x, map_waypoints_y);
						new_car_x_waypoints.push_back(new_car_xy[0]);
						new_car_y_waypoints.push_back(new_car_xy[1]);
					}

					// Transform into car coordinates. 
					auto new_car_carxy_waypoints = GlobalToCarTransform(new_car_x_waypoints,
						new_car_y_waypoints, ref_x, ref_y, ref_yaw);
					std::cout << "This is printing new_car_carxy_waypoints[0] : " << std::endl;
					printVector(new_car_carxy_waypoints[0]);
					std::cout << "This is printing new_car_carxy_waypoints[1] : " << std::endl;
					printVector(new_car_carxy_waypoints[1]);

					// - Create x and y interpolated reference points:
					tk::spline spline_xy_car;   // this is in car coordinates align with heading
					spline_xy_car.set_points(new_car_carxy_waypoints[0], new_car_carxy_waypoints[1]);
					double new_x_car;
					double new_y_car;
					vector<double> new_xy_global;
					double delta_x_car = set_speed * dT;   // Assuming car_yaw does not change much in one horizon
					auto starting_xy_car = GlobalToCarTransform(ref_x, ref_y, ref_x, ref_y, ref_yaw);
					int l = next_x_vals.size();
					for (int i = 1; i <= T / dT - l; i++) {
						new_x_car = i * delta_x_car + starting_xy_car[0];
						new_y_car = spline_xy_car(new_x_car);
						// Transform back to global coordinates
						new_xy_global = SE2Transform(new_x_car, new_y_car, ref_x, ref_y, ref_yaw);
						next_x_vals.push_back(new_xy_global[0]);
						next_y_vals.push_back(new_xy_global[1]);
					}
					std::cout << "This is printing next_x_vals : " << std::endl;
					printVector(next_x_vals);
					std::cout << "This is printing next_y_vals : " << std::endl;
					printVector(next_y_vals);





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
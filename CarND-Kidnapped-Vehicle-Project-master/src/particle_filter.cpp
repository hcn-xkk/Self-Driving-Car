/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h" // Need this for sampling from distributions
// need to use std::normal_distribution;


using std::string;
using std::vector;
std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles
  fill(weights.begin(), weights.end(), 1.0/ num_particles);   // initialize to even weights
  
  // Create normal distribution for each state:
  
  std::normal_distribution<double> pos_x(x, std[0]);
  std::normal_distribution<double> pos_y(y, std[1]);
  std::normal_distribution<double> pos_theta(theta, std[2]);

  // Initialize particles locations by GPS measurements with random noise.
  for (int i = 0; i < num_particles; i++) {
	  Particle p;
	  p.x = pos_x(gen);
	  p.y = pos_y(gen);
	  p.theta = pos_theta(gen);
	  p.weight = weights[i];
	  particles.push_back(p);

	  // The following members remain unchanged. 
	  /*particles[i].associations;
	  particles[i].sense_x;
	  particles[i].sense_y;*/
  }

  // Set state var initilzed.
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
	/*
	# turn, and add randomness to the turning command
	orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
	orientation %= 2 * pi
	# move, and add randomness to the motion command
	dist = float(forward) + random.gauss(0.0, self.forward_noise)
	x = self.x + (cos(orientation) * dist)
	y = self.y + (sin(orientation) * dist)
	x %= world_size    # cyclic truncate
	y %= world_size
	*/

	// std::default_random_engine gen;
	std::normal_distribution<double> x_distribution(0.0, std_pos[0]);
	std::normal_distribution<double> y_distribution(0.0, std_pos[1]);
	std::normal_distribution<double> theta_distribution(0.0, std_pos[2]);

	for (int i = 0; i < num_particles; i++) {
		particles[i].theta += yaw_rate * delta_t + theta_distribution(gen) + 2 * M_PI;
		while (particles[i].theta >= 2 * M_PI) {
			particles[i].theta -= 2 * M_PI;
		}
		double dist = delta_t * velocity;
		particles[i].x += dist * cos(particles[i].theta) + x_distribution(gen);
		particles[i].y += dist * sin(particles[i].theta) + y_distribution(gen);
	}
	
}

vector<int> ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
	const Map &map_landmarks) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
	// Initialize nearest_landmarks_ids
	vector<int> nearest_landmarks_ids(predicted.size(), 0);
	for (int i = 0; i < predicted.size(); i++) {
		// Calculate distance of all landmarks to the landmark observation predicted[i]
		vector <double> dist_arr_i;
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			dist_arr_i.push_back( dist(predicted[i].x, predicted[i].y, 
				map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f) );
		}

		// Find the index of the nearest landmark 
		nearest_landmarks_ids[i] = findMinElementIndex(dist_arr_i);
	}

	return nearest_landmarks_ids;

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
	vector<LandmarkObs> obs_map_coord_j;
	vector<int> nearest_landmarks_ids;
	for (int j = 0; j < particles.size(); j++) {

		// Record the association.
		for (int i = 0; i < observations.size(); i++) {
			particles[j].associations.push_back(observations[i].id);
			particles[j].sense_x.push_back(observations[i].x);
			particles[j].sense_y.push_back(observations[i].y);
		}
		
		// Change observation to map coordinates 
		obs_map_coord_j = changeCoordinates(particles[j], observations);

		// Association: Get the indices in map markers for the nearest landmarks for each observation in obs_map_coord_j.
		nearest_landmarks_ids = dataAssociation(obs_map_coord_j, map_landmarks);
		
		// Calculate and update weights[j]
		particles[j].weight = multiv_prob_vector(obs_map_coord_j, map_landmarks, nearest_landmarks_ids, std_landmark);
		weights[j] = particles[j].weight;
	}

}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   * Random number distribution that produces integer values according to a discrete distribution, 
   * where each possible value has a predefined probability of being produced
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
	
	// std::default_random_engine generator;
	// Create the distribution with those weights
	std::discrete_distribution<int> distribution_weights(weights.begin(), weights.end());

	// Create a temporary var: resampled_particles
	vector <Particle> resampled_particles;
	for (int i = 0; i < num_particles; i++) {
		int index = distribution_weights(gen);
		resampled_particles.push_back(particles[index]);
	}
	particles = resampled_particles;

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
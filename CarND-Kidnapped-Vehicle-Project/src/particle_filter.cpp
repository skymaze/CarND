/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	if (is_initialized) {
		cout << "Filter Already Initialized" << endl;
		return;
	}

	cout << "Init Filter" << endl;

	// Create normal (Gaussian) distributions for x y theta

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	// Number particle filter
	num_particles = 10;

	// Initialize all particle
	for (int i = 0; i < num_particles; ++i) {
		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1;

		// Append new particle to particles list
		weights.push_back(1.0);
		particles.push_back(particle);

		// Print particle to the terminal.
		cout << "Partcle " << i << " x:" << particle.x << " y:" << particle.y << " theta:" << particle.theta << endl;
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	// Define normal distributions for sensor noise
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for (int i = 0; i < num_particles; i++) {
		// Calculate new state
		if (fabs(yaw_rate) < 0.0001) {
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		} else {
			particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}

		// Add noise
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	int l_observations = observations.size();
	int l_predictions = predicted.size();

	for (int i = 0; i < l_observations; i++) {
		// Get current observation
		LandmarkObs landmarkObs = observations[i];

		// Init minimum distance to maximum
		double min_dist = numeric_limits<double>::max();

		// Init id of landmark from map placeholder to associate with the observation
		int map_id = -1;

		for (int j = 0; j < l_predictions; j++) {
			// Get current prediction
			LandmarkObs landmark_predict = predicted[j];

			// Get distance between current and predicted landmarks
			double distance = dist(landmarkObs.x, landmarkObs.y, landmark_predict.x, landmark_predict.y);

			// Find the predicted landmark nearest the current observed landmark
			if (distance < min_dist) {
				min_dist = distance;
				map_id = landmark_predict.id;
			}
		}

		// Update the observation identifier
		observations[i].id = map_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	// For each partical
	for (int i = 0; i < num_particles; i++) {
		double std_x = std_landmark[0];
		double std_y = std_landmark[1];

		for (int i = 0; i < num_particles; i++) {

			double x = particles[i].x;
			double y = particles[i].y;
			double theta = particles[i].theta;

			// Find landmarks in particle's range.
			double sensorRangeSquare = sensor_range * sensor_range;
			vector<LandmarkObs> predict;
			for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
				float landmarX = map_landmarks.landmark_list[j].x_f;
				float landmarY = map_landmarks.landmark_list[j].y_f;
				int id = map_landmarks.landmark_list[j].id_i;
				double dX = x - landmarX;
				double dY = y - landmarY;

				if (dX * dX + dY * dY <= sensorRangeSquare) {
					predict.push_back(LandmarkObs{id, landmarX, landmarY});
				}
			}

			// Transform observation coordinates.
			vector<LandmarkObs> mapObs;
			for (int j = 0; j < observations.size(); j++) {
				double obsX = observations[j].x;
				double obsY = observations[j].y;
				double transformX = cos(theta) * obsX - sin(theta) * obsY + x;
				double transformY = sin(theta) * obsX + cos(theta) * obsY + y;
				mapObs.push_back(LandmarkObs{observations[j].id, transformX, transformY});
			}

			// Observation association to landmark.
			dataAssociation(predict, mapObs);

			// Reseting weight.
			particles[i].weight = 1.0;

			// Calculate weights.
			for (int j = 0; j < mapObs.size(); j++) {
				double obsX = mapObs[j].x;
				double obsY = mapObs[j].y;

				int landmarkId = mapObs[j].id;

				double landmarkX, landmarkY;
				int k = 0;
				int landmarks = predict.size();
				bool findLandmark = false;
				while (k < landmarks && !findLandmark) {
					if (predict[k].id == landmarkId) {
						findLandmark = true;
						landmarkX = predict[k].x;
						landmarkY = predict[k].y;
					}
					k++;
				}

				// Calculating weight.
				double dX = obsX - landmarkX;
				double dY = obsY - landmarkY;
				double multiplier = 1 / (0.5 * M_PI * std_x * std_y);
				double divider = 0.5 * std_x * std_y;
				double divider2 = 0.5 * pow(std_y, 2);
				double weight = multiplier * exp(-(dX * dX / divider + (dY * dY / divider2)));
				particles[i].weight *= weight;
			}
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	vector<Particle> new_particles;

	// Get all of the current weights
	vector<double> weights;
	for (int i = 0; i < num_particles; i++) {
		weights.push_back(particles[i].weight);
	}

	// Generate random starting index for resampling wheel
	uniform_int_distribution<int> uniintdist(0, num_particles - 1);
	auto index = uniintdist(gen);

	// Get max weight
	double max_weight = *max_element(weights.begin(), weights.end());

	// Uniform random distribution
	uniform_real_distribution<double> unirealdist(0.0, max_weight);

	double beta = 0.0;

	// Spin the resample wheel
	for (int i = 0; i < num_particles; i++) {
		beta += unirealdist(gen) * 2.0;
		while (beta > weights[index]) {
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		new_particles.push_back(particles[index]);
	}

	particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

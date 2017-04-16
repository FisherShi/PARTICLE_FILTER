/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <map>
#include <numeric>

#include "particle_filter.h"
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    //set number of particles
    num_particles = 100;
    //cout << "number of particles: "<<num_particles << endl;

    double weight = 1;
    Particle particle;

    // noise generation
    default_random_engine gen;
    normal_distribution<double> N_x_init(0, std[0]);
    normal_distribution<double> N_y_init(0, std[1]);
    normal_distribution<double> N_theta_init(0, std[2]);
    double noise_x, noise_y, noise_theta;

    for (int i = 0; i < num_particles; ++i){

        // random noises
        noise_x = N_x_init(gen);
        noise_y = N_y_init(gen);
        noise_theta = N_theta_init(gen);

        //assign values to particles
        particle.id = i;
        particle.x = x + noise_x;
        particle.y = y + noise_y;
        particle.theta = theta + noise_theta;
        particles.push_back(particle);

        //initialize all the weights to 1
        weights.push_back(weight);
    }
    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    // noise generation
    default_random_engine gen;
    normal_distribution<double> N_x_init(0, std_pos[0]);
    normal_distribution<double> N_y_init(0, std_pos[1]);
    normal_distribution<double> N_theta_init(0, std_pos[2]);
    double noise_x, noise_y, noise_theta;

    double x_p, y_p, theta_p;
    for (int i = 0; i < num_particles; ++i){
        double x = particles[i].x;
        double y = particles[i].y;
        double theta = particles[i].theta;

        // random noises
        noise_x = N_x_init(gen);
        noise_y = N_y_init(gen);
        noise_theta = N_theta_init(gen);

        //if yaw_rate is not too small
        if (abs(yaw_rate) > 0.001) {
            //predict x,y,theta
            x_p = x +  (velocity/yaw_rate) * ((sin(theta+delta_t*yaw_rate)) - (sin(theta)));
            y_p = y +  (velocity/yaw_rate) * ((cos(theta)) - (cos(theta+delta_t*yaw_rate)));
            theta_p = theta + yaw_rate*delta_t;
        }
        //if yaw_rate is too small
        else{
            x_p = x + velocity * cos(theta) * delta_t;
            y_p = y + velocity * sin(theta) * delta_t;
            theta_p = theta + yaw_rate*delta_t;
        }
        //assign new values to particles
        particles[i].x = x_p + noise_x;
        particles[i].y = y_p + noise_y;
        particles[i].theta = theta_p + noise_theta;
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

    double x_std = std_landmark[0];
    double y_std = std_landmark[1];
    double x_std_sqr = std_landmark[0]*std_landmark[0];
    double y_std_sqr = std_landmark[1]*std_landmark[1];

    double weight_sum = 0;

    for (int i = 0; i < num_particles; ++i) {
        //reset the weight
        particles[i].weight = 1;
        //transform the observations from the vehicle's coordinate system to MAP's coordinate system
        double x_particle = particles[i].x;
        double y_particle = particles[i].y;
        double theta_particle = particles[i].theta;

        //create a small map within sensor range
        Map map_small;
        for (int j=0; j<map_landmarks.landmark_list.size();++j){
            double x_landmark = map_landmarks.landmark_list[j].x_f;
            double y_landmark = map_landmarks.landmark_list[j].y_f;
            double x_distance = (x_particle - x_landmark) * (x_particle - x_landmark);
            double y_distance = (y_particle - y_landmark) * (y_particle - y_landmark);
            double distance = sqrt(x_distance + y_distance);
            if (distance < sensor_range){
                map_small.landmark_list.push_back(map_landmarks.landmark_list[j]);
            }
        }

        //run through each observation of a particle and match with the small map
        for (int j = 0; j < observations.size(); ++j) {
            //convert from car coordinate to MAP coordinate system
            double x_obs_car = observations[j].x;
            double y_obs_car = observations[j].y;
            double x_obs_map = x_particle + (x_obs_car * cos(theta_particle) - y_obs_car * sin(theta_particle));
            double y_obs_map = y_particle + (x_obs_car * sin(theta_particle) + y_obs_car * cos(theta_particle));

            //match landmark based on distance
            double distance_min = 100;
            double x_distance_min = 100;
            double y_distance_min = 100;
            int landmark_match;

            for (int k = 0; k < map_small.landmark_list.size(); ++k) {
                double x_landmark = map_small.landmark_list[k].x_f;
                double y_landmark = map_small.landmark_list[k].y_f;
                double x_distance = (x_obs_map - x_landmark) * (x_obs_map - x_landmark);
                double y_distance = (y_obs_map - y_landmark) * (y_obs_map - y_landmark);
                double distance = sqrt(x_distance + y_distance);
                //update minimum distance
                if (distance < distance_min) {
                    distance_min = distance;
                    x_distance_min = x_distance;
                    y_distance_min = y_distance;
                    landmark_match = map_small.landmark_list[k].id_i;
                }
            }
            observations[j].id = landmark_match;

            //update weight based on the distance between each observation and its matching landmark
            double w = exp(-0.5*(x_distance_min/x_std_sqr+y_distance_min/y_std_sqr))/(2*M_PI*x_std*y_std);
            particles[i].weight = particles[i].weight*w;
        }
        weight_sum = weight_sum + particles[i].weight;
        }
    //assign the weights
    for (int i=0; i < num_particles; ++i){
        //normalize weights
        particles[i].weight = particles[i].weight/weight_sum;
        weights[i] = particles[i].weight;
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    random_device rd;
    mt19937 gen(rd());
    discrete_distribution<> d(weights.begin(),weights.end());
    vector<Particle> temp_particles;
    //resample
    for(int i=0; i<num_particles; ++i) {
        Particle a = particles[d(gen)];
        temp_particles.push_back(a);
    }
    //update particles
    for(int i=0; i<num_particles; ++i) {
        particles[i] = temp_particles[i];
    }
}


void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}

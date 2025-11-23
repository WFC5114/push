/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"
using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    weights.clear();
    particles.clear();

    num_particles = 729; // set to number of files in observation directory

    weights.resize(num_particles);
    particles.resize(num_particles);

    double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];

    // Normal distribution for x, y and theta
    normal_distribution<double> dist_x(x, std_x); // mean is centered around the new measurement
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

     // http://www.cplusplus.com/reference/random/default_random_engine/

    // create particles and set their values
    for (int i = 0; i < num_particles; ++i) {
        Particle p;
        p.id = i;
        p.x = dist_x(gen); // take a random value from the Gaussian Normal distribution and update the attribute
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1;

        particles[i] = p;
        weights[i] = p.weight;
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
    std_x = std_pos[0];
    std_y = std_pos[1];
    std_theta = std_pos[2];


    for (int i = 0; i < num_particles; ++i) {
        Particle* p = &particles[i]; // get address of particle to update

        // use the prediction equations from the Lesson 14
        if (fabs(yaw_rate) > 1e-5) {
            p->x += (velocity / yaw_rate) * (sin(p->theta + yaw_rate * delta_t) - sin(p->theta));
            p->y += (velocity / yaw_rate) * (cos(p->theta) - cos(p->theta + yaw_rate * delta_t));
            p->theta += yaw_rate * delta_t;
        } else {
            p->x += velocity * delta_t * cos(p->theta);
            p->y += velocity * delta_t * sin(p->theta);
            // p->theta += yaw_rate * delta_t; // yaw_rate is zero, so no change to theta
        }

        // add Gaussian Noise to each measurement
        // Normal distribution for x, y and theta
        normal_distribution<double> dist_x(p->x, std_x);
        normal_distribution<double> dist_y(p->y, std_y);
        normal_distribution<double> dist_theta(p->theta, std_theta);

        // update the particle attributes
        p->x = dist_x(gen);
        p->y = dist_y(gen);
        p->theta = dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    for (auto &obs : observations) {          
        double min_dist = std::numeric_limits<double>::max();
        int map_id = -1;

        for (auto &pred : predicted) {
            double distance = dist(obs.x, obs.y, pred.x, pred.y);
            if (distance < min_dist) {
                min_dist = distance;
                map_id = pred.id;
            }
        }
        obs.id = map_id;  
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations,
                                   Map map_landmarks) {
    // Update the weights of each particle using a multi-variate Gaussian distribution. You can read
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

    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    double weights_sum = 0;

    for (int i = 0; i < num_particles; ++i) {
        Particle* p = &particles[i];
        double wt = 1.0;

        // convert observation from vehicle's to map's coordinate system
        for (int j = 0; j < observations.size(); ++j) {
            LandmarkObs current_obs = observations[j];
            LandmarkObs transformed_obs;

            transformed_obs.x = (current_obs.x * cos(p->theta)) - (current_obs.y * sin(p->theta)) + p->x;
            transformed_obs.y = (current_obs.x * sin(p->theta)) + (current_obs.y * cos(p->theta)) + p->y;
            transformed_obs.id = current_obs.id;

            // Assume transformed_obs is defined with fields x and y,
            // and that 'map' is our constant Map instance

            double distance_min = std::numeric_limits<double>::max();
            Map::single_landmark_s landmark;

            // Check distance to each landmark (obstacle)
            for (size_t k = 0; k < map_landmarks.landmark_list.size(); ++k) {
                Map::single_landmark_s cur_l = map_landmarks.landmark_list[k];
                double distance = dist(transformed_obs.x, transformed_obs.y, cur_l.x_f, cur_l.y_f);
                if (distance < distance_min) {
                    distance_min = distance;
                    landmark = cur_l;
                }
            }

            // Check distance to the boundaries
            double d_left = fabs(transformed_obs.x - map_landmarks.min_x);
            double d_right = fabs(map_landmarks.max_x - transformed_obs.x);
            double d_bottom = fabs(transformed_obs.y - map_landmarks.min_y);
            double d_top = fabs(map_landmarks.max_y - transformed_obs.y);

            // Get the minimum boundary distance
            double boundary_distance = std::min({d_left, d_right, d_bottom, d_top});

            // If the distance to the boundary is smaller, update the minimum distance and landmark info
            if (boundary_distance < distance_min) {
                distance_min = boundary_distance;

                // Optionally, create a special "boundary" landmark.
                // Here, we set id = 0 to indicate a boundary, and we choose the boundary point
                // that is closest to transformed_obs.
                if (boundary_distance == d_left) {
                    landmark.x_f = map_landmarks.min_x;
                    landmark.y_f = transformed_obs.y;
                } else if (boundary_distance == d_right) {
                    landmark.x_f = map_landmarks.max_x;
                    landmark.y_f = transformed_obs.y;
                } else if (boundary_distance == d_bottom) {
                    landmark.x_f = transformed_obs.x;
                    landmark.y_f = map_landmarks.min_y;
                } else if (boundary_distance == d_top) {
                    landmark.x_f = transformed_obs.x;
                    landmark.y_f = map_landmarks.max_y;
                }
            }

            // Now, distance_min holds the minimum distance to either an obstacle or the boundary,
            // and 'landmark' holds the corresponding information.

            // update weights using Multivariate Gaussian Distribution
            // equation given in Transformations and Associations Quiz
            double num = exp(-0.5 * (pow((transformed_obs.x - landmark.x_f), 2) / pow(std_x, 2) +
                                     pow((transformed_obs.y - landmark.y_f), 2) / pow(std_y, 2)));
            double denom = 2 * M_PI * std_x * std_y;
            wt *= num / denom;
        }
        weights_sum += wt;
        p->weight = wt;
    }
    // normalize weights to bring them in (0, 1]
    if (weights_sum < 1e-10) weights_sum = 1e-10;  // 防止除零
    for (int i = 0; i < num_particles; i++) {
        Particle* p = &particles[i];
        p->weight /= weights_sum;
        weights[i] = p->weight;
    }
}

void ParticleFilter::resample() {
    // Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


    // Random integers on the [0, n) range
    // the probability of each individual integer is its weight of the divided by the sum of all weights.
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<Particle> resampled_particles;

    for (int i = 0; i < num_particles; i++) { resampled_particles.push_back(particles[distribution(gen)]); }

    particles = resampled_particles;
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
Pose ParticleFilter::getBestParticlePose() const {
    double max_w = -1;
    const Particle* best = &particles[0];
    for (const auto &p : particles) {
        if (p.weight > max_w) {
            max_w = p.weight;
            best = &p;
        }
    }
    return {best->x, best->y, best->theta};
}

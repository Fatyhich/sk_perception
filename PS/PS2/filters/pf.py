"""
Sudhanva Sreesha
ssreesha@umich.edu
28-Mar-2018

This file implements the Particle Filter.
"""

import numpy as np
from numpy.random import uniform
from scipy.stats import norm as gaussian

from filters.localization_filter import LocalizationFilter
from tools.task import get_gaussian_statistics
from tools.task import get_observation
from tools.task import sample_from_odometry
from tools.task import wrap_angle


class PF(LocalizationFilter):
    def __init__(self, initial_state, alphas, bearing_std, num_particles, global_localization):
        super(PF, self).__init__(initial_state, alphas, bearing_std)
        
        # TODO add here specific class variables for the PF
        self.num_particles = num_particles
        
        if global_localization:
            # Uniform distribution over the map
            self.particles = np.zeros((self.num_particles, 3))
            self.particles[:,0] = uniform(0, 10, self.num_particles) 
            self.particles[:,1] = uniform(0, 10, self.num_particles)
            self.particles[:,2] = uniform(-np.pi, np.pi, self.num_particles)
        else:
            # Distribution around initial_state
            self.particles = np.zeros((self.num_particles, 3))
            self.particles[:,0] = gaussian.rvs(loc=initial_state.mu[0],
                                scale=np.sqrt(initial_state.Sigma[0,0]),
                                size=self.num_particles)
            self.particles[:,1] = gaussian.rvs(loc=initial_state.mu[1],
                                scale=np.sqrt(initial_state.Sigma[1,1]),
                                size=self.num_particles)
            self.particles[:,2] = gaussian.rvs(loc=initial_state.mu[2],
                                scale=np.sqrt(initial_state.Sigma[2,2]),
                                size=self.num_particles)

        self.weights = np.ones(num_particles) / num_particles

    @property
    def X(self):
        """
        Returns the particles as a numpy array of shape (3, N), 
        where N is the number of particles
        """
        return self.particles

    def predict(self, u):
        # TODO Implement here the PF, perdiction part
        for i in range(self.num_particles):
            self.particles[i] = sample_from_odometry(self.particles[i], u, self._alphas)

        state = get_gaussian_statistics(self.particles)
        # self._state_bar.mu = state.mu[np.newaxis].T
        self._state_bar.mu = state.mu
        self._state_bar.Sigma = state.Sigma

    def low_variance_resample(self):
        num_particles = self.num_particles
        resampled_particles = np.zeros_like(self.particles)
        
        # Starting point
        random_start = uniform(0, 1.0 / num_particles)
        cumulative_weight = self.weights[0]
        particle_index = 0

        # Systematic resampling
        for new_particle_idx in range(num_particles):
            sampling_point = random_start + new_particle_idx / num_particles
            
            while sampling_point > cumulative_weight and particle_index < num_particles - 1:
                particle_index += 1
                cumulative_weight += self.weights[particle_index]
                
            resampled_particles[new_particle_idx] = self.particles[particle_index]

        # Update
        self.particles = resampled_particles.copy()
        self.weights.fill(1.0 / num_particles)

    def update(self, z):
        # TODO implement correction step
        for i in range(self.num_particles):
            observation_bar = get_observation(self.particles[i], int(z[1]))
            innovation = wrap_angle(z[0] - observation_bar[0])
            self.weights[i] = self.weights[i] * gaussian.pdf(innovation, 0, np.sqrt(self._Q))

        self.weights = self.weights / np.sum(self.weights)
        self.low_variance_resample()

        state = get_gaussian_statistics(self.particles)
        # self._state_bar.mu = state.mu[np.newaxis].T
        self._state.mu = state.mu
        self._state.Sigma = state.Sigma

#!/usr/bin/env python
from __future__ import division

from threading import Lock

import numpy as np


class LowVarianceSampler:
    """Low-variance particle sampler."""

    def __init__(self, particles, weights, state_lock=None):
        """Initialize the particle sampler.

        Args:
            particles: the particles to update
            weights: the weights to update
            state_lock: guarding access to the particles and weights during update,
                since both are shared variables with other processes
        """
        self.particles = particles
        self.weights = weights
        self.state_lock = state_lock or Lock()
        self.n_particles = particles.shape[0]

        # You may want to cache some intermediate variables here for efficiency

    def resample(self):
        """Resample particles using the low-variance sampling scheme.

        Both self.particles and self.weights should be modified in-place.
        """
        # Acquire the lock that synchronizes access to the particles. This is
        # necessary because self.particles is shared by the other particle
        # filter classes.
        #
        # The with statement automatically acquires and releases the lock.
        # See the Python documentation for more information:
        # https://docs.python.org/3/library/threading.html#using-locks-conditions-and-semaphores-in-the-with-statement
        with self.state_lock:
            # BEGIN QUESTION 3.2
            M = self.n_particles
            
            # 1. Create the evenly spaced pointers
            # r is a single random number between 0 and 1/M
            step = 1.0 / M
            r = np.random.uniform(0, step)
            
            # Create an array of M pointers spaced exactly 'step' apart
            pointers = r + np.arange(M) * step
            
            # 2. Create the intervals by calculating the cumulative sum of weights
            # For example, weights [0.2, 0.5, 0.3] become intervals [0.2, 0.7, 1.0]
            cum_weights = np.cumsum(self.weights)
            
            # 3. Find which interval each pointer falls into
            # searchsorted acts as our O(M) lookup, instantly matching pointers to particles
            indices = np.searchsorted(cum_weights, pointers)
            
            # 4. Update the arrays strictly IN-PLACE
            # We use slice assignment [:] to overwrite the particles in memory
            self.particles[:] = self.particles[indices]
            
            # Reset all weights back to a uniform distribution in-place
            self.weights.fill(step)
            # END QUESTION 3.2

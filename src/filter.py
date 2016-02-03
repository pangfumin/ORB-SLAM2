# -*- coding: utf-8 -*-
"""
Created on Wed Feb  3 11:35:58 2016

@author: sujiwo
"""

from __future__ import division
import numpy as np
from copy import copy


class Observation:
    def __init__ (self, observation):
        pass


class Particle:
    def __init__ (self, stateInitFunc):
        self.w = 0
        self.state = stateInitFunc()



class ParticleFilter:
    
    def __init__ (self, numOfParticles, stateInitFunc, motionModelFunc, measurementModelFunc):
        self.motion = motionModelFunc
        self.measurement = measurementModelFunc
        # generate and initialize particles
        self.particles = []
        for i in range(numOfParticles):
            p = Particle (stateInitFunc)
            self.particles.append(p)
    
    def update (self, control, observation):
        # Prediction
        for particle in self.particles:
            particle.prevState = copy (particle.state)
            self.motion (particle.state, control)
            
        # Update
            

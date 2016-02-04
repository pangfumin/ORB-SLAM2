# -*- coding: utf-8 -*-
"""
Created on Wed Feb  3 11:35:58 2016

@author: sujiwo
"""

from __future__ import division
import numpy as np
from copy import copy
import random


class Particle:
    def __init__ (self, stateInitFunc=None):
        self.w = 0
        if (stateInitFunc is not None):
            self.state = stateInitFunc()



class ParticleFilter:
    
    def __init__ (self, numOfParticles, stateInitFunc, motionModelFunc, measurementModelFunc):
        self.numOfParticles = numOfParticles
        self.motion = motionModelFunc
        self.measurement = measurementModelFunc
        # generate and initialize particles
        self.particles = []
        for i in range(numOfParticles):
            p = Particle (stateInitFunc)
            self.particles.append(p)
    
    def update (self, control, observation=None):
        # Prediction
        for particle in self.particles:
            particle.prevState = copy (particle.state)
            self.motion (particle.state, control)
            
        # Update
        if (observation==None):
            return
        # 1: Importance factor
        w_all = 0
        for particle in self.particles:
            particle.w = self.measurement (particle.state, observation)
            w_all += particle.w
        # 2: Resampling
        r = random.random() / float(self.numOfParticles)
        i = 0
        c = 0
        particles_new = [Particle() for ip in range(self.numOfParticles)]
        for m in range(self.numOfParticles) :
            U = r + m / float(self.numOfParticles)
            while (U > c):
                i+=1
                c += self.particles[i] / w_all
                particles_new[m].state = self.particles[i].state
        

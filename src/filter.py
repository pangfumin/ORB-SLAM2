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
            self.prevState = stateInitFunc()
        else:
            self.state = None
            self.prevState = None       
        
    def swapState (self):
        self.state, self.prevState = self.prevState, self.state


def nrand (num) :
    r = num * np.sqrt(-2.0*np.log(random.random())) * np.cos(2.0*np.pi*random.random())
    return r


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
            # Motion model should return new copy of state
            particle.state = self.motion (particle.prevState, control)
            particle.swapState()
            
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
        c = self.particles[0].w / w_all
        for m in range(self.numOfParticles) :
            U = r + m / float(self.numOfParticles)
            while (U > c):
                i+=1
                c += self.particles[i].w / w_all
            self.particles[m].state = self.particles[i].prevState
        for m in range(self.numOfParticles) :
            self.particles[m].swapState()
            
    def getStates (self):
        st = [p.state for p in self.particles]
        return st
        

# -*- coding: utf-8 -*-
"""
Created on Mon Dec  7 10:29:59 2015

@author: sujiwo
"""
import numpy as np
import matplotlib.pyplot as plt



filename = '/tmp/orb-slam-ndt.csv'
orbcolsg = [0, 1, 2]
orbcols = [11, 12, 13]
ndtcols = [14, 15, 16]

table = np.loadtxt(filename, delimiter=',', skiprows=1)
orbresult = np.loadtxt (filename, delimiter=',', skiprows=1, usecols=orbcolsg)
ndtresult = np.loadtxt (filename, delimiter=',', skiprows=1, usecols=ndtcols)
#plt.plot (ndtresult[:,0], ndtresult[:,1], ',')
plt.plot (orbresult[:,0], orbresult[:,2], ',')

pass
#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 23 11:11:19 2016

@author: sujiwo
"""

from orbndt import PoseTable
from sys import argv


if __name__ == '__main__' :
    if len(argv) < 4:
        print ("Usage:")
        print ("setOffsetBagTime.py <source bag> <destination bag> <time_difference>")
        exit(-1)
    orbSrc = PoseTable.loadFromBagFile(argv[1])
    orbFrame = list(PoseTable.getFrameList(argv[1]))[0]
    
    timeDif = float(argv[3])
    
    for p in orbSrc.table:
        p.timestamp += timeDif
    
    orbSrc.saveToBag(argv[2], orbFrame[0], orbFrame[1])
    print ("From '{}' to '{}'".format(orbFrame[0], orbFrame[1]))
    print ("Saved succesfully")

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 27 15:03:13 2020

@author: agathe
"""
import numpy as np
from copy import deepcopy
from math import pi

def definePossibleActions(wind_speed,wind_angle, dead_zone_range):
    directions = {0:"E", 1:"NE", 2:"N", 3:"NW", 4:"W", 5:"SW", 6:"S", 7:"SE"}
    actions = {}
    if wind_speed !=0:
        for key in directions.keys():
            angle = key*pi/4-wind_angle-pi
            if not(abs(angle) < dead_zone_range):
                actions[key] = directions[key]
    return(actions)

def defineStates(distance_matrix):
    states = {}
    k = len(distance_matrix)
    for i in range(len(distance_matrix)):
        for j in range(len(distance_matrix[0])):
            states[(i,j)] = i*k + j
    return(states)

def moveSailboat(state,action):
    realAction = [k for k in actions.values()][actions]
    # TODO : attributes rewards
    return(newState,reward)

wind_angle = 0
wind_speed = 2
deadZoneRange =  1.39 # 80 degrees
distMatrix = np.array([[1,0,1,2],
                     [2,1,2,3],
                     [3,2,3,4],
                     [4,3,4,5]])

rewardMatrix = deepcopy(np.max(distMatrix)-distMatrix)
           
states = defineStates(distMatrix)  
actions =  definePossibleActions(wind_speed, wind_angle, deadZoneRange)

Q = np.zeros((len(states), len(actions)))
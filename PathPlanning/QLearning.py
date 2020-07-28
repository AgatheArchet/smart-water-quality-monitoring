#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 27 15:03:13 2020

@author: agathe
"""
import numpy as np
from copy import deepcopy
from math import pi
import random
from QLearning_plot import Grid

def defineActions(wind_speed,wind_angle, dead_zone_range):
    
    directions = {0:"E", 1:"NE", 2:"N", 3:"NW", 4:"W", 5:"SW", 6:"S", 7:"SE"}
    actions = []
    if wind_speed !=0:
        for key in directions.keys():
            angle = key*pi/4-(wind_angle)%(2*pi)
            if not(abs(angle) < dead_zone_range):
                actions.append(key)
    return(actions)

def defineStates(distance_matrix):
    
    states = []
    k = len(distance_matrix[0])
    for i in range(len(distance_matrix)-1,-1,-1):
        states.append([])
        for j in range(k):
            states[k-i].append(i*k + j)
    return(states)

class EnvGrid(object):
    
    def __init__(self, distanceMatrix, start, wind_speed, wind_angle, deadZoneRange):
        super(EnvGrid, self).__init__()
        
        self.grid = distanceMatrix
        self.reward = deepcopy(np.max(distMatrix)-distMatrix)
        self.xlim = distanceMatrix.shape[1] -1
        self.ylim = distanceMatrix.shape[0] -1
        
        # starting position
        self.x0, self.y0 = start[0], start[1]
        self.x, self.y = start[0], start[1]
        
        # states
        self.states = defineStates(distanceMatrix)
        
        # possible actions 
        self.directions = [[1,0],   # E
                            [1,1],   # NE
                            [0,1],   # N
                            [-1,1],  # NW
                            [-1,0],  # W
                            [-1,-1], # SW
                            [0,-1],  # S
                            [1,-1]]  # SE
        self.realActions =  defineActions(wind_speed, wind_angle, deadZoneRange)
        
    def reset(self):
        self.x = self.x0
        self.y = self.y0
        return(self.getState())
        
    def getGridCoords(self):
        posx, posy = self.ylim-self.y-1, self.x
        return(posx,posy)
    
    def getState(self):
        posx, posy = self.getGridCoords()
        return(self.states[posx][posy])
    
    def getReward(self):
        posx, posy = self.getGridCoords()
        return(self.reward[posx][posy])
    
    def step(self, action):
        
        move = self.directions[self.realActions[action]]
        self.x = max(0, min(self.x + move[0],self.xlim))
        self.y = max(0, min(self.y + move[1],self.ylim))
        posx, posy = self.getGridCoords()
        return (self.getState() , self.getReward())
    
    def is_finished(self):
        return(self.getReward() == self.reward.max())
    
    def show(self):
        pass
    
def take_action(st, Q, eps):
    # Take an action
    if random.uniform(0, 1) < eps:
        action = random.randint(0,len(Q[0])-1)
    else: # Or greedy action
        action = np.argmax(Q[st])
    return action

if __name__=='__main__':
    
    wind_angle = 0
    wind_speed = 2
    deadZoneRange =  1.22 # 80 degrees
    
    distMatrix = np.array([[1,0,1,2],
                         [2,1,2,3],
                         [3,2,3,4],
                         [4,3,4,5],
                         [5,4,5,6]])
    
    env = EnvGrid(distMatrix, (1,0), wind_speed, wind_angle, deadZoneRange)
    Q = np.zeros(((max(max(env.states)))+1, len(env.realActions)))
    
    for _ in range(100):
        # Reset the game
        st = env.reset()
        while not env.is_finished():
            #env.show()
            #at = int(input("$>"))
            at = take_action(st, Q, 0.4)
    
            stp1, r = env.step(at)
            #print("s", stp1)
            #print("r", r)
    
            # Update Q function
            atp1 = take_action(stp1, Q, 0.0)
            Q[st][at] = Q[st][at] + 0.1*(r + 0.9*Q[stp1][atp1] - Q[st][at])
    
            st = stp1
            
    for s in range(1, len(Q)):
        print(s, Q[s])
    
    grid = Grid(distMatrix)
    grid.addColors()
    grid.show()
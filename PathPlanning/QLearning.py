#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 27 15:03:13 2020

@author: agathe
"""
import numpy as np
from copy import deepcopy, copy
from math import pi
import random
from QLearning_plot import Grid

def defineActions(wind_speed,wind_angle, dead_zone_range):
    
    directions = {0:"E", 1:"NE", 2:"N", 3:"NW", 4:"W", 5:"SW", 6:"S", 7:"SE"}
    actions = []
    impossible = []
    if wind_speed !=0:
        for key in directions.keys():
            angle = key*pi/4-(wind_angle)%(2*pi)
            if not(abs(angle) < dead_zone_range):
                actions.append(key)
            else:
                impossible.append(key)
    return(actions,impossible)

def defineStates(area):
    
    states = []
    k = len(area[0])
    for i in range(len(area)-1,-1,-1):
        states.append([])
        for j in range(k):
            states[len(area)-1-i].append(i*k + j)
    return(states)

def defineRewardDistance(matrix, goal):
    reward = np.zeros(matrix.shape)
    ymax, xmax = matrix.shape
    goal = (ymax-goal[1],goal[0]+1)
    for i in range(ymax):
        for j in range(xmax):
            reward[i,j] = abs(i-goal[0]) + abs(j-goal[1])
    reward = np.max(reward)-reward
    for i in range(ymax):
        for j in range(xmax):
            if matrix[i,j] == -1:
                reward[i,j] =- 1000
    return(reward)

def defineRewardClassic(matrix, goal):
    reward = - 5*np.ones(matrix.shape)
    ymax, xmax = matrix.shape
    for i in range(ymax):
        for j in range(xmax):
            if matrix[i,j] < 0:
                reward[i,j] =- 100
    reward[ymax-goal[1], goal[0]] = 25
    return(reward)

class EnvGrid(object):
    
    def __init__(self, Matrix, start, end, wind_speed, wind_angle, deadZoneRange):
        super(EnvGrid, self).__init__()
        
        self.grid = Matrix
        #self.reward = defineRewardDistance(Matrix, end)
        self.reward = defineRewardClassic(Matrix, end)
        self.xlim = Matrix.shape[1] -1
        self.ylim = Matrix.shape[0] -1
        
        # main positions
        self.x0, self.y0 = start[0], start[1]
        self.x, self.y = start[0], start[1]
        self.xEnd, self.yEnd = end[0], end[1]
        
        # states
        self.states = defineStates(Matrix)
        
        # possible actions 
        self.directions = [[1,0],   # E
                           [1,1],   # NE
                           [0,1],   # N
                           [-1,1],  # NW
                           [-1,0],  # W
                           [-1,-1], # SW
                           [0,-1],  # S
                           [1,-1]]  # SE
        self.realActions, self.impo =  defineActions(wind_speed, wind_angle, deadZoneRange)
        
    def reset(self):
        self.x = self.x0
        self.y = self.y0
        return(self.getState())
        
    def getGridCoords(self):
        """
        converts coordinates of the boat to get their equivalents on the
        distance and the reward matrices (where (x_mat, y_mat) = (ymax-y,x))
        """
        posx, posy = self.ylim-self.y, self.x
        return(posx,posy)
    
    def getState(self):
        """
        gives the state associated with the boat's position.
        """
        posx, posy = self.getGridCoords()
        return(self.states[posx][posy])
    
    def getReward(self):
        """
        gives the reward associated with the boat's position.
        """
        posx, posy = self.getGridCoords()
        return(self.reward[posx][posy])
    
    def step(self, action):
        """
        performes an action and return the new observation of the environment
        (new state and reward)

        """
        move = self.directions[self.realActions[action]]
        self.x = max(0, min(self.x + move[0],self.xlim))
        self.y = max(0, min(self.y + move[1],self.ylim))
        newState, newReward = self.getState(), self.getReward()
        return (newState, newReward)
    
    def is_finished(self):
        return((self.x == self.xEnd) and (self.y==self.yEnd))
    
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
    
    # environement parameters
    wind_angle = 0
    wind_speed = 2
    deadZoneRange =  1.22 # 80 degrees
    
    # Map = np.array([[0,0,0,0],
    #                 [0,0,0,0],
    #                 [0,0,0,0],
    #                 [0,0,0,0],
    #                 [0,0,0,-1]])
    
    Map = np.array([[-1,-1,-1,-1,-1,-1,-1,-1],
                      [-1,-1,0,0,0,0,-1,-1],
                      [-1,0,0,0,0,0,0,-1],
                      [-1,0,0,0,0,0,0,-1],
                      [-1,0,0,0,0,0,-1,-1],
                      [-1,0,0,0,0,0,0,-1],
                      [-1,-1,0,0,0,0,0,-1],
                      [-1,0,0,0,0,0,-1,-1]])
    
    # Q-Learning parameters
    α = 0.1 # learning rate
    γ = 0.9 # discount factor
    ε = 0.4 # random explorer
    
    # Q-Learning tools
    env = EnvGrid(Map, (3,0), (4,6), wind_speed, wind_angle, deadZoneRange)
    Q = np.zeros(((max(max(env.states)))+1, len(env.realActions)))
    
    # Plot
    grid = Grid(Map)
    episodes = 200
    grid.plotMap([env.getState(),52])
    grid.plotQTable(Q)
    score = []
    path = []
    for _ in range(episodes):
        # Reset the game
        st = env.reset()
        score.append(0)
        path = [st]
        while not env.is_finished():
            #env.show()
            #at = int(input("$>"))
            at = take_action(st, Q, ε)
    
            stp1, r = env.step(at)
            #print("s", stp1)
            #print("r", r)
    
            # Update Q function
            atp1 = take_action(stp1, Q, 0.0)
            Q[st][at] = Q[st][at] + α*(r + γ *Q[stp1][atp1] - Q[st][at])
    
            st = stp1
            score[-1] = score[-1] + 1 + min(0,r)
            path.append(st)
        # plotting
        grid.plotMap(path)
        grid.plotReward(range(len(score)),score)
        
        Qplot = copy(Q)
        for i in env.impo:
            Qplot = np.insert(Qplot, i, 0, axis=1)
        grid.plotQTable(Qplot)
        
        print(str(_)) 
        
    for s in range(1, len(Q)):
        print(s, Q[s])
    
    print(path)
    
    grid.show()
    
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

def defineActions(wind_speed, wind_angle, dead_zone_range):
    """
    defines feasible and impossible actions (directions) that the boat can 
    perform without moving against the wind. All actions are allowed if there 
    is no wind.
        
    0: East | 1: North-East | 2: North | 3: North-West 
    4: West | 5: South-West | 6: South | 7: South-East
    
    Parameters
    ----------
    wind_speed : positive float in m/s
    wind_angle : float (radians)
    dead_zone_range : float (radians)
    """
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
    """
    numbers all cells of the area to define unique states for the Q-Table.
    """
    states = []
    k = len(area[0])
    for i in range(len(area)-1,-1,-1):
        states.append([])
        for j in range(k):
            states[len(area)-1-i].append(i*k + j)
    return(states)

def defineRewardDistance(matrix, goal):
    """
    creates a reward matrix based on distance between cells. The goal has the 
    highest possible reward (acts like a climb-gradient function).
    
    eg.
    
    matrix =  0  0 -1   with goal (0,0) gives reward =   5    4 -1000
              0  0  0                                    4    3   2
             -1  0  0                                  -1000  2   1
             
    Parameters
    ----------
    
    matrix : numpy array
        matrix where obstacle cells equal -1 and other cells equal 0.
    goal : tuple
    """
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
    """
    creates a reward matrix based on the nature of cells. The goal is highly
    rewarded, the obstacles are sanctionned.

    eg.
    
    matrix =  0  0 -1   with goal (0,0) gives reward =  25   0 -100
              0  0  0                                    0   0   0
             -1  0  0                                  -100  0   0
    
    Parameters
    ----------
    
    matrix : numpy array
        matrix where obstacle cells equal -1 and other cells equal 0.
    goal : tuple
    """
    reward = - 5*np.ones(matrix.shape)
    ymax, xmax = matrix.shape
    for i in range(ymax):
        for j in range(xmax):
            if matrix[i,j] == -1:
                reward[i,j] =- 100
    reward[ymax-goal[1], goal[0]] = 25
    return(reward)

class EnvGrid(object):
    """
    A class that modelizes the environment in which the Q-Learning agent 
    evolves.
    
    Attributes
    ----------
    
    grid : numpy matrix
        the matrix of the environement by default.
    reward : numpy matrix
        the associated reward matrix.
    xlim,ylim : floats
        shape of the area.
    x0, y0 : floats
        coordinates of initial state.
    x, y : floats
        coordinates of current state.
    xEnd, YEnd : floats
        coordinates of the goal to reach.
    states: nested list of integers
        list of the different states, callable by their coordinates.
    directions : list of list of integers
        contains the change in coordinates when performing a specific action.
    realActions : list of integers
        list of allowed actions due to the wind constraints.
    impo : list of integers
        complementary list of realActions, with all impossible actions for the
        agent.
    sol : None / list of integers
        path with the states that lead to the most optimal solution found.
        None by default, of if there is no solution yet.
    
    Methods
    -------
    
    reset()
    getGridCoords()
    getState(other)
    getReaward()
    step(action)
    is_finished()
    isThereASolution(Q)
    getOptimalPath(Q)
    
    """
    def __init__(self, Matrix, start, end, wind_speed, wind_angle, deadZoneRange):
        super(EnvGrid, self).__init__()
        
        self.grid = Matrix
        self.reward = defineRewardDistance(Matrix, end)
        #self.reward = defineRewardClassic(Matrix, end)
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
        
        # final solution
        self.sol = None
        
    def reset(self):
        """
        reinitialises to initial position, and returns inital state.
        """
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
    
    def getState(self, other=None):
        """
        gives the state associated with the boat's position.
        """
        if other == None:
            posx, posy = self.getGridCoords()
        else:
            posx, posy = self.ylim-other[1], other[0]
        return(self.states[posx][posy])
    
    def getReward(self):
        """
        gives the reward associated with the boat's position.
        """
        posx, posy = self.getGridCoords()
        return(self.reward[posx][posy])
    
    def step(self, action):
        """
        performes an action and returns the new observation of the environment
        (new state and reward)

        """
        move = self.directions[self.realActions[action]]
        self.x = max(0, min(self.x + move[0],self.xlim))
        self.y = max(0, min(self.y + move[1],self.ylim))
        newState, newReward = self.getState(), self.getReward()
        return (newState, newReward)
    
    def is_finished(self):
        return((self.x == self.xEnd) and (self.y==self.yEnd))
    
    def isThereASolution(self,Q):
        """
        tests if the Q-Table values are different enough to find an optimal 
        path.
        """
        exist = True
        state = self.reset()
        solution = [state]
        while not self.is_finished():
            max_action = take_action(state, Q, 0)
            state, r = self.step(max_action)
            if state in solution:
                exist = False
                solution.append(state)
                break
            else:
                solution.append(state)
        return(exist) 
    
    def getOptimalPath(self, Q):
        """
        returns list of states if an optimal path is found at the moment.
        """
        state = self.reset()
        solution = [state]
        scoring = 0
        if self.isThereASolution(Q):
            while not self.is_finished():
                max_action = take_action(state, Q, 0)
                state, r = self.step(max_action)
                scoring = scoring + 1 + min(0,r)
                solution.append(state)
            return(solution,scoring)
        else:
            return(None, None)
    
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
    wind_speed = 5
    deadZoneRange =  1.22 # 80 degrees
    
    start = (3,1)
    goal = (4,7)
    
    Map = np.array([[-1,-1,-1,-1,-1,-1,-1,-1,-1],
                      [-1,-1,0,0,0,0,0,-1,-1],
                      [-1,0,0,0,0,0,0,0,-1],
                      [-1,0,0,0,0,0,0,0,-1],
                      [-1,0,0,0,0,0,0,-1,-1],
                      [-1,0,0,0,0,0,0,0,-1],
                      [-1,-1,0,0,0,0,0,0,-1],
                      [-1,0,0,0,0,0,0,-1,-1],
                      [-1,-1,-1,-1,-1,-1,-1,-1,-1]])
    
    # Q-Learning parameters
    α = 0.8 # learning rate 
    γ = 0.7 # discount factor
    ε = 0.3 # random explorer
    episodes = 1000
    
    # Q-Learning tools
    env = EnvGrid(Map, start, goal, wind_speed, wind_angle, deadZoneRange)
    Q = np.zeros(((max(max(env.states)))+1, len(env.realActions)))
    
    # Plot
    grid = Grid(Map)
    grid.plotMap([env.getState(), env.getState(goal)])
    grid.plotQTable(Q)
    score = []
    path = []
    
    for _ in range(episodes):
        # Reset the environment
        st = env.reset()
        score.append(0)
        path = [st]
        while not env.is_finished():
            at = take_action(st, Q, ε)
            stp1, r = env.step(at)
            # Update Q function
            atp1 = take_action(stp1, Q, 0.0)
            Q[st][at] = Q[st][at] + α*(r + γ *Q[stp1][atp1] - Q[st][at])
            #save values
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
    
    print(env.getOptimalPath(Q))
    grid.show()
    
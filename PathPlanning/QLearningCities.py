#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Aug 27 9:14:20 2020

@author: agathe
"""
import numpy as np
from copy import deepcopy, copy
from math import pi
import random
from QLearning_plotCities import Grid

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
    xmax = matrix.shape[1] -1
    ymax = matrix.shape[0] -1
    goalMap = []
    for g in goal :
        goalMap.append((ymax-g[1],g[0]))
    for i in range(ymax+1):
        for j in range(xmax+1):
            mapReward = []
            for g in goalMap:
                 mapReward.append(abs(i-g[0])+abs(j-g[1]))
            reward[i,j] = min(mapReward)
    reward = np.max(reward)-reward
    for i in range(ymax+1):
        for j in range(xmax+1):
            if matrix[i,j] == -1:
                reward[i,j] =- 1000
    return(reward)

def defineRewardClassic(matrix, goal):
    """
    creates a reward matrix based on the nature of cells. The goal is highly
    rewarded, the obstacles are sanctionned.

    eg.
    
    matrix =  0  0 -1   with goal (0,0) gives reward =  100   1 -100
              0  0  0                                    -1  -1   -1
             -1  0  0                                  -100  -1   -1
    
    Parameters
    ----------
    
    matrix : numpy array
        matrix where obstacle cells equal -1 and other cells equal 0.
    goal : tuple
    """
    reward = - 1*np.ones(matrix.shape)
    xmax = matrix.shape[1] -1
    ymax = matrix.shape[0] -1
    for i in range(ymax+1):
        for j in range(xmax+1):
            if matrix[i,j] == -1:
                reward[i,j] =- 100
    for city in goal:
        reward[ymax-city[1], city[0]] = 100
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
    
    def resetReward(self):
        self.reward = deepcopy(self.reward0)
        
    def removeCityClassic(self,city):
        x,y = env.getGridCoords(city)
        self.reward[x,y] = -1
        
    def removeCityGradient(self,list_visited_cities,goal):
        cities_to_visit = []
        for city in goal:
            if (not(city in list_visited_cities)):
                cities_to_visit.append((city[0],city[1]))
        self.reward = defineRewardDistance(self.grid, cities_to_visit)
            
        
    def getGridCoords(self, other=None):
        """
        converts coordinates of the boat to get their equivalents on the
        distance and the reward matrices (where (x_mat, y_mat) = (ymax-y,x))
        """
        if other == None:
            posx, posy = self.ylim-self.y, self.x
        else: 
            posx,posy = self.ylim-other[1], other[0]
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
    
    def getReward(self, other=None):
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
    
    def isInCity(self,cities):
        inCity = False
        for city in cities:
            if (self.x == city[0]) and (self.y == city[1]):
                inCity = True
                break
        return(inCity)
    
    def isThereASolution(self,Q,cities):
        """
        tests if the Q-Table values are different enough to find an optimal 
        path.
        """
        exist = True
        state = self.reset()
        solution = [state]
        visited_cities = []
        while len(list(set(visited_cities))) != len(cities):
            max_action = take_action(state, Q, 0)
            state, r = self.step(max_action)
            if self.isInCity(cities):
                visited_cities.append(state)
            if state in solution:
                exist = False
                solution.append(state)
                break
            else:
                solution.append(state)
        print(solution)
        state = self.reset()
        return(exist) 
    
    def getOptimalPath(self, Q,goal):
        """
        returns list of states if an optimal path is found at the moment.
        """
        state = self.reset()
        solution = [state]
        scoring = 0
        visited_points = []
        if self.isThereASolution(Q,goal):
            while len(list(set(visited_points))) != len(goal):
                max_action = take_action(state, Q, 0)
                state, r = self.step(max_action)
                scoring = scoring + 1 + min(0,r)
                solution.append(state)
                if self.isInCity(goal):
                    visited_points.append(state)
            return(solution,scoring)
        else:
            return([], 0)
        
    def show(self,grid,Q,cities,path,score,finalPath=False):
        grid.plotMap(cities,path,finalPath)
        grid.plotReward(range(len(score)),score)
        Qplot = copy(Q)
        for i in env.impo:
            Qplot = np.insert(Qplot, i, 0, axis=1)
        grid.plotQTable(Qplot)
    
def take_action(st, Q, eps):
    # Take an action
    if random.uniform(0, 1) < eps:
        action = random.randint(0,len(Q[0])-1)
    else: # Or greedy action
        action = np.argmax(Q[st])
    return action

if __name__=='__main__':
    
#------------------------Customize this area-----------------------------------

    # environement parameters
    wind_angle = 0
    wind_speed = 5
    deadZoneRange =  1.22 # 80 degrees
    
    start = (3,1)
    goal = [(1,4),(4,7),(6,2)]
    
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
    α = 0.5 # learning rate 0.3
    γ = 0.8 # discount factor  0.7
    ε = 0.3 # random explorer 0.3
    episodes = 10000
    
    # plotting parameters
    plotEachStep = False
    qmin, qmax = -10, 45 # range for Q-Table coloring 
    rmin,rmax = None,None # range for reward graph if needed
    
#------------------------------------------------------------------------------    
    
    # Q-Learning tool
    env = EnvGrid(Map, start, goal, wind_speed, wind_angle, deadZoneRange)
    nb_cities = len(goal)
#-----------------------------Choose the reward system-------------------------

    # distance rewarding
    Q = np.zeros(((max(max(env.states)))+1, len(env.realActions)))
    for city in goal:
         Q[env.getState(city)] = len(env.realActions)*[1000]
    env.reward0 = defineRewardDistance(env.grid, goal)
    env.reward = deepcopy(env.reward0)
    system = "gradient"    
    
    # classic rewarding
    #Q = np.zeros(((max(max(env.states)))+1, len(env.realActions)))
    # env.reward0 = defineRewardClassic(env.grid, goal)
    # env.reward = deepcopy(env.reward0)
    # system = "classic"
#------------------------------------------------------------------------------
    
    # Init graphical tools
    grid = Grid(Map, qmin, qmax,rmin,rmax)
    score = [0]
    state_cities = [env.getState(start)] + [env.getState(city) for city in goal]
    env.show(grid,Q,state_cities,state_cities,score)  
    path = []
    
    for episode in range(episodes):
        # Reset the environment
        st = env.reset()
        env.resetReward()
        path = [st]
        score.append(score[-1])
        visited_cities = []
        visited_cities_coords = []
        while  len(list(set(visited_cities))) != len(goal):
            at = take_action(st, Q, ε)
            st_new, r = env.step(at)
            # Update Q function
            atp_max = take_action(st_new, Q, 0.0)
            # if episode > 9999:
            #print("st: "+str(st_new)+" ac: "+str(at)+" r: "+str(Q[st][at] + α*(r + γ *Q[st_new][atp_max] - Q[st][at])))
            Q[st][at] = Q[st][at] + α*(r + γ *Q[st_new][atp_max] - Q[st][at])
            try:
                if st == visited_cities[-1]:
                    if system=="classic":
                        env.removeCityClassic(visited_cities_coords[-1])
                    if system=="gradient":
                        env.removeCityGradient(visited_cities_coords,goal)
            except:
                pass
            #save values
            score[-1] += α*(r + γ *Q[st_new][atp_max] - Q[st][at])
            st = st_new
            path.append(st)
            if env.isInCity(goal):
                if not (st in visited_cities):
                    visited_cities.append(st)
                    print(visited_cities)
                    visited_cities_coords.append((env.x,env.y))
        # plotting
        if plotEachStep:
            env.show(grid,Q,state_cities,path,score)
    
        print(episode) 
    
    # Final results
    for s in range(1, len(Q)):
        print(s, Q[s])
        
    optimalPath,length = env.getOptimalPath(Q,goal)
    if len(optimalPath)!=0:
        env.show(grid,Q,state_cities,optimalPath,score,finalPath=True)
        print("A solution has been found! One optimal possible path is :")
        print(optimalPath)
    else:
        env.show(grid,Q,state_cities,state_cities,score)
        print("No solution have been found yet (presence of loops in Q-Table)...")
    grid.show()
    
    
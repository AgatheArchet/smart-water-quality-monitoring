#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 28 14:04:28 2020

@author: agathe
"""

import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from matplotlib.ticker import MaxNLocator

class Grid:
    
    def __init__(self, distMatrix):
        
        # create a 8" x 8" board
        gridsize = (2,3)
        fig = plt.figure(figsize=([13, 8]))
        plt.suptitle("Q-Learning algorithm strategy", fontsize=20)
        
        # main structures
        self.ax1 = plt.subplot2grid(gridsize, (0, 0), colspan=2, rowspan=2)
        self.ax2 = plt.subplot2grid(gridsize, (0, 2))
        self.ax3 = plt.subplot2grid(gridsize, (1, 2))
        self.ax3.set_title("Q-Table")
        self.ax2.axes.get_xaxis().set_visible(False)
        self.ax2.axes.get_yaxis().set_visible(False)
        self.ax3.set_title("Score through time")
        self.ax3.axes.get_xaxis().set_visible(False)
        self.ax3.axes.get_yaxis().set_visible(False)
        
        # matrix of the map
        self.Map = distMatrix
        self.xmin = 0
        self.xmax = len(distMatrix[0])
        self.ymin = 0
        self.ymax = len(distMatrix)
    
        # scale the plot area conveniently
        self.ax1.set_xlim(self.xmin-0.5,self.xmax-0.5)
        self.ax1.set_ylim(self.ymin-0.5,self.ymax-0.5)
        self.ax1.xaxis.set_major_locator(MaxNLocator(integer=True))
        self.ax1.yaxis.set_major_locator(MaxNLocator(integer=True))
        
        

    def addColors(self):
        for i in range(0,self.xmax):
            for j in range(0,self.ymax):
                if self.Map[j,i] == 0:
                    color = "yellowgreen"
                if self.Map[j,i] > 0:
                    color = "lightseagreen"     
                if self.Map[j,i] <=-10:
                    color = "lightblue"
                self.ax1.add_artist(
                    patches.Rectangle((i-0.5,self.ymax-j-1.5), 1, 1,
                                      facecolor = color,
                                      fill = True))
                    
    def plotReward(self,list_episode,list_reward):
        self.ax3.cla()
        self.ax3.plot(list_episode,list_reward)
        plt.pause(2)
    
    def show(self):
        plt.show()
    
if __name__=='__main__':
    
    distMatrix = np.array([[-1000,-1000,-1000,-1000,-1000,-1000,-1000,-1000],
                         [-1000,-1000,3,2,1,2,-1000,-1000],
                         [-1000,3,2,1,0,1,2,-1000],
                         [-1000,4,3,2,1,2,3,-1000],
                         [-1000,5,4,3,2,3,-1000,-1000],
                         [-1000,6,5,4,3,4,5,-1000],
                         [-1000,-1000,6,5,4,5,6,-1000],
                         [-1000,8,7,6,5,7,-1000,-1000]])

    grid = Grid(distMatrix)
    grid.addColors()
    grid.plotReward([1,2,3],[1.26,1.39,1.8])
    grid.plotReward([1,2,3,4],[1.26,1.39,1.8,4])
    grid.show()
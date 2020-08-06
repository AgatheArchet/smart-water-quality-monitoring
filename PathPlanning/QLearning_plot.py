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
    
    def __init__(self, distMatrix, Qmin, Qmax, Rmin=None, Rmax=None):
        
        # matrix of the map
        self.Map = distMatrix
        self.xmin = 0
        self.xmax = len(distMatrix[0])
        self.ymin = 0
        self.ymax = len(distMatrix)
        
        # Q-Table value range
        self.qmin = Qmin
        self.qmax = Qmax
        
        # reward graph value range
        self.rmin = Rmin
        self.rmax = Rmax
        
        # create a 8" x 8" board
        gridsize = (3,3)
        self.fig = plt.figure(figsize=([10, 6]))
        # main structures
        self.ax1 = plt.subplot2grid(gridsize, (0, 0), colspan=2, rowspan=3)
        self.ax2 = plt.subplot2grid(gridsize, (0, 2), colspan=1, rowspan=2)
        self.ax3 = plt.subplot2grid(gridsize, (2, 2))
        self.ax1.set_title("Q-Learning algorithm strategy", fontsize=15)
        self.ax2.set_title("Q-Table")
        mapy = plt.get_cmap('coolwarm')
        mapy.set_under('grey')
        im = self.ax2.imshow(np.asmatrix(np.zeros((self.xmax*self.ymax-1,8))), 
                             interpolation='nearest', cmap=mapy, vmin=self.qmin, 
                             vmax=self.qmax, aspect='auto')
        self.ax2.set_yticks(np.arange(0,self.xmax*self.ymax-1,10))
        self.ax2.set_yticklabels([str(i) for i in range(0,self.xmax*self.ymax-1,10)])
        self.fig.colorbar(im,ax=self.ax2, extend='min')
        #self.ax2.axes.get_xaxis().set_visible(False)
        #self.ax2.axes.get_yaxis().set_visible(False)
        self.ax3.set_title("Accumulated score over time")
        #self.ax3.axes.get_xaxis().set_visible(False)
        #self.ax3.axes.get_yaxis().set_visible(False)
    
    
        # scale the plot area conveniently
        self.ax1.set_xlim(self.xmin-0.5,self.xmax-0.5)
        self.ax1.set_ylim(self.ymin-0.5,self.ymax-0.5)
        self.ax1.xaxis.set_major_locator(MaxNLocator(integer=True))
        self.ax1.yaxis.set_major_locator(MaxNLocator(integer=True))
        plt.tight_layout()
        

    def plotMap(self,listOfStates = [], finalPath=False):
        self.ax1.cla()
        self.ax1.set_title("Q-Learning algorithm strategy", fontsize=15)
        self.ax1.set_xlim(self.xmin-0.5,self.xmax-0.5)
        self.ax1.set_ylim(self.ymin-0.5,self.ymax-0.5)
        self.ax1.xaxis.set_major_locator(MaxNLocator(integer=True))
        self.ax1.yaxis.set_major_locator(MaxNLocator(integer=True))
        
        for i in range(0,self.xmax):
            for j in range(0,self.ymax):
                # if self.Map[j,i] == 0:
                #     color = "yellowgreen"
                if self.Map[j,i] == 0:
                    color = "lightseagreen"   
                if self.Map[j,i] ==-1:
                    color = "grey"
                self.ax1.add_artist(
                    patches.Rectangle((i-0.5,self.ymax-j-1.5), 1, 1,
                                      facecolor = color,
                                      fill = True))
        
        if len(listOfStates)>0:
            finalx,finaly = [],[]
            for k in range(len(listOfStates)):
                if (k == 0 or k == len(listOfStates)-1 or 
                    listOfStates[k]==listOfStates[0]) :
                    color = "green"
                    #print("green : "+str(listOfStates[k]))
                    if finalPath:
                        finalx.append(listOfStates[k]%(self.xmax))
                        finaly.append(listOfStates[k]//(self.xmax))
                else :
                    if finalPath:
                        finalx.append(listOfStates[k]%(self.xmax))
                        finaly.append(listOfStates[k]//(self.xmax))
                        color = 'lightseagreen'
                    else:
                        color = "orange"
                posy = self.ymax -1 -listOfStates[k]//(self.xmax)
                posx = listOfStates[k]%(self.xmax)
                self.ax1.add_artist(
                        patches.Rectangle((posx-0.5,self.ymax-posy-1.5), 1, 1,
                                          facecolor = color,fill = True))
                
        for i in range(0,self.xmax+1):
            for j in range(0, self.ymax+1):
                self.ax1.text(j, i, i*(self.xmax) +j,
                        ha="center", va="center", color="w", fontsize = 6)
        if finalPath:
            self.ax1.plot(finalx,finaly)
        plt.pause(0.01)
        
    def plotReward(self,list_episode,list_reward):
        self.ax3.cla()
        self.ax3.set_title("Accumulated score over time")
        self.ax3.plot(list_episode,list_reward)
        self.ax3.xaxis.set_major_locator(MaxNLocator(integer=True))
        if self.rmin !=None:
            self.ax3.set_ylim([self.rmin,self.rmax])
        plt.pause(0.01)
        
    def plotQTable(self, Qtable):
        self.ax2.cla()
        self.ax2.set_title("Q-Table")
        self.ax2.set_yticks(np.arange(0,len(Qtable),10))
        self.ax2.set_yticklabels([str(i) for i in range(0,len(Qtable),10)])
        self.ax2.set_xticks(np.arange(0,8))
        self.ax2.set_xticklabels(["E","NE","N","NW","W","SW","S","SE"])
        mapy = plt.get_cmap('coolwarm')
        mapy.set_under('grey')
        im = self.ax2.imshow(np.asmatrix(Qtable), interpolation='nearest',
                      cmap=mapy, vmin=self.qmin, vmax=self.qmax, aspect='auto')
        plt.pause(0.01)
        
    def show(self):
        plt.show()
    
    
if __name__=='__main__':
    
    # distMatrix = np.array([[-1,-1,-1,-1,-1,-1,-1,-1,-1],
    #                       [-1,-1,0,0,0,0,0,-1,-1],
    #                       [-1,0,0,0,0,0,0,0,-1],
    #                       [-1,0,0,0,0,0,0,0,-1],
    #                       [-1,0,0,0,0,0,0,-1,-1],
    #                       [-1,0,0,0,0,0,0,0,-1],
    #                       [-1,-1,0,0,0,0,0,0,-1],
    #                       [-1,0,0,0,0,0,0,-1,-1]])
    
    # distMatrix = np.array([[-1,-1,-1,-1,-1,-1,-1,-1],
    #                       [-1,-1,0,0,0,0,-1,-1],
    #                       [-1,0,0,0,0,0,0,-1],
    #                       [-1,0,0,0,0,0,0,-1],
    #                       [-1,0,0,0,0,0,-1,-1],
    #                       [-1,0,0,0,0,0,0,-1],
    #                       [-1,-1,0,0,0,0,0,-1],
    #                       [-1,0,0,0,0,0,-1,-1]])
    
    distMatrix = np.array([[-1,-1,-1,-1,-1,-1,-1,-1],
                          [-1,-1,0,0,0,0,-1,-1],
                          [-1,0,0,0,0,0,0,-1],
                          [-1,0,0,0,0,0,0,-1],
                          [-1,0,0,0,0,0,-1,-1],
                          [-1,0,0,0,0,0,-1,-1],
                          [-1,0,0,0,0,0,0,-1],
                          [-1,-1,0,0,0,0,0,-1],
                          [-1,0,0,0,0,0,-1,-1]])

    grid = Grid(distMatrix,-20,50)
    grid.plotMap()
    grid.plotReward([1,2,3],[1.26,1.39,1.8])
    #grid.plotQTable(np.array([[2.3,0,0.3,-5.7],[0.65,-2,0.3,0.75]]))
    q = np.random.randint(-5,5, size=(64, 8))
    q[0,0]= -1000
    grid.plotQTable(q)
    grid.plotReward([1,2,3,4],[1.26,1.39,1.8,4])
    q[0,0] = 2
    grid.plotQTable(q)
    grid.plotMap([3,10,17])
    grid.show()
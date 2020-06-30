#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 30 08:26:21 2020

@author: agathe
"""
from graph import doIntersect
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy

def isInsidePolygon(n,Vx,Vy,x,y):
    """
    Jordan curve Theorem
    """
    c = False
    for i in range(0,n):
        j = (i-1)%n
        if ((Vy[i]>y)!=(Vy[j]>y)) and (x < (Vx[j]-Vx[i])*(y-Vy[i])/(Vy[j]-Vy[i])+Vx[i]):
            c = not(c)
    return c 

def isNearlyInsidePolygon(n,Vx,Vy,x,y,delta):
    sign = [-1,0,1]
    c = False
    for q in sign:
        for p in sign:
            c = c or isInsidePolygon(n,Vx,Vy,x+delta*q,y+delta*p)
    return c
                


#x,y = [0,1,3,2.5,6,5,4],[0,1,6,1,0.5,4,1]
obstacle = [[3,3.25,3.5,3.5,3.5,3.5,3,3,3],[0.5,0,0.5,1,1.5,2,2,1.5,1]]

x,y = [],[]
for i in np.linspace(2,4,4):
    for j in np.linspace(0,6,10):
        if not(isNearlyInsidePolygon(len(obstacle[0]),obstacle[0],obstacle[1],i,j,0.3)):
            x.append(i)
            y.append(j)
xmin,xmax,ymin,ymax  = min(x)-0.2,max(x)+0.2,min(y)-0.2,max(y)+0.2


points =deepcopy([[obstacle[0][i],obstacle[1][i]] for i in range(len(obstacle[0]))])
points2 =  [[x[i],y[i]] for i in range(len(x))]
frame = [[xmin,y] for y in np.linspace(ymin,ymax,20)]
frame += [[xmax,y] for y in np.linspace(ymin,ymax,20)]
frame += [[x,ymin] for x in np.linspace(xmin,xmax,20)]
frame += [[x,ymax] for x in np.linspace(xmin,xmax,20)]
points = points + frame + points2

vor = Voronoi(points)
# voronoi vertices
for simplex in vor.ridge_vertices:
    simplex = np.asarray(simplex)
    if np.all(simplex >= 0):
        vorx = vor.vertices[simplex, 0]
        vory = vor.vertices[simplex, 1]
        n = len(obstacle[0])
        for i in range(n):
            if doIntersect((vorx[0],vory[0]), (vorx[1],vory[1]),(obstacle[0][i],obstacle[1][i]),(obstacle[0][(i+1)%n],obstacle[1][(i+1)%n])):
                print("ok")
        plt.plot(vorx,vory,'k-')
        plt.plot(vorx[0],vory[0],'ro')

# vertices
plt.plot(x[:],y[:],'o',color="grey")
        
# obstacle
plt.plot(obstacle[0]+[obstacle[0][0]],obstacle[1]+[obstacle[1][0]],color="blue")
        
       
plt.xlim([xmin-0.5,xmax+0.5])
plt.ylim([ymin-0.5,ymax+0.5])
plt.show()

#print(vor.vertices)


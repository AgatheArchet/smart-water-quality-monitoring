#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 30 08:26:21 2020

@author: agathe
"""
from graph import doIntersect
from scipy.spatial import Voronoi
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
        if (((Vy[i]>y)!=(Vy[j]>y)) and 
            (x < (Vx[j]-Vx[i])*(y-Vy[i])/(Vy[j]-Vy[i])+Vx[i])):
            c = not(c)
    return c 

def isNearlyInsidePolygon(n,Vx,Vy,x,y,delta):
    sign = [-1,0,1]
    c = False
    for q in sign:
        for p in sign:
            c = c or isInsidePolygon(n,Vx,Vy,x+delta*q,y+delta*p)
    return c
 
def generateFrame(x,y):
    xmin,xmax,ymin,ymax  = min(x)-0.2,max(x)+0.2,min(y)-0.2,max(y)+0.2
    frame = [[xmin,y] for y in np.linspace(ymin,ymax,len(set(y))+1)]
    frame += [[xmax,y] for y in np.linspace(ymin,ymax,len(set(y))+1)]
    frame += [[x,ymin] for x in np.linspace(xmin,xmax,len(set(x))+1)]
    frame += [[x,ymax] for x in np.linspace(xmin,xmax,len(set(x))+1)]
    return(frame)
    
def generateVoronoi(points,obstacles):
    vor = Voronoi(points)
    for simplex in vor.ridge_vertices:
        simplex = np.asarray(simplex)
        if np.all(simplex >= 0):
            vorx = vor.vertices[simplex, 0]
            vory = vor.vertices[simplex, 1]
            intersect, inside1,inside2 = False, False, False
            for obstacle in obstacles:
                n = len(obstacle[0])
                for i in range(n):
                    if doIntersect((vorx[0],vory[0]), (vorx[1],vory[1]),
                                   (obstacle[0][i],obstacle[1][i]),
                                   (obstacle[0][(i+1)%n],obstacle[1][(i+1)%n])):
                        intersect = True
                        break
                    elif isInsidePolygon(n,obstacle[0],obstacle[1],vorx[0],vory[0]):
                        intersect = True
                        break
                if isInsidePolygon(n,obstacle[0],obstacle[1],vorx[0],vory[0]):
                    inside1 = True
                if isInsidePolygon(n,obstacle[0],obstacle[1],vorx[1],vory[1]):
                    inside2 = True    
            if not(intersect):
               plt.plot((vorx[0],vorx[1]),(vory[0],vory[1]),'y-')
            else:
                #plt.plot((vorx[0],vorx[1]),(vory[0],vory[1]),'r-')
                pass
            if not(inside1):
                #plt.plot(vorx[0],vory[0],'*',color="orange")
                pass
            if not(inside2):
                #plt.plot(vorx[1],vory[1],'*',color="orange")
                pass
    #plt.plot((vorx[0],vorx[1]),(vory[0],vory[1]),'y-',label="Voronoi diagram")
    

if __name__=='__main__':

    obstacles = [[[3,3.25,3.5,3.5,3.5,3.5,3,3,3],[0.5,0,0.5,1,1.5,2,2,1.5,1]],
                [[0,1,0],[3,4,4]]]
    
    x,y = [],[]
    for i in np.linspace(-1,6,8):
        for j in np.linspace(-1,8,10):
            for obstacle in obstacles:
                if not(isNearlyInsidePolygon(len(obstacle[0]),obstacle[0],obstacle[1],i,j,0.3)):
                    x.append(i)
                    y.append(j)
    
    points = []
    for obstacle in obstacles :
        a = deepcopy([[obstacle[0][i],obstacle[1][i]] for i in range(len(obstacle[0]))])
        points += a    
    frame = generateFrame(x,y)
    points = points + frame
    
    # voronoi vertices
    generateVoronoi(points,obstacles)
            
    # obstacle
    for obstacle in obstacles:
        plt.plot(obstacle[0]+[obstacle[0][0]],obstacle[1]+[obstacle[1][0]],color="blue", label="obstacle")
        plt.plot(obstacle[0]+[obstacle[0][0]],obstacle[1]+[obstacle[1][0]],"*b") 
        
    #frame
    plt.plot(np.array(frame)[:,0],np.array(frame)[:,1],'*g',label="frame")
       
    plt.xlim([xmin-0.5,xmax+0.5])
    plt.ylim([ymin-0.5,ymax+0.5])
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', ncol=1,borderaxespad=0.)
    plt.show()
    
    #print(vor.vertices)


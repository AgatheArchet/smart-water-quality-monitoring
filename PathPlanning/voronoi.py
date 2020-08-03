#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 30 08:26:21 2020

@author: agathe
"""
from graph import doIntersect, Graph, euclideanDistance
from scipy.spatial import Voronoi, ConvexHull
import matplotlib.pyplot as plt
import numpy as np
from copy import deepcopy
from math import pi
from boat import *

def extractDuplicate(a):
    b = list(set(a))
    for elem in b:
        a.remove(elem)
    return(a)

def isInsidePolygon(n,Vx,Vy,x,y):
    """
    verifies is a given point at coords (x,y) is inside a given polygon (Vx,Vy)
    of n summits (based upon the Jordan curve theorem).
    """
    c = False
    for i in range(0,n):
        j = (i-1)%n
        if (((Vy[i]>y)!=(Vy[j]>y)) and 
            (x < (Vx[j]-Vx[i])*(y-Vy[i])/(Vy[j]-Vy[i])+Vx[i])):
            c = not(c)
    return c 

def isNearlyInsidePolygon(n,Vx,Vy,x,y,delta):
    """
    verifies is a given point at coords (x,y) is inside a given polygon (Vx,Vy)
    of n summits, with a delta tolerance (based upon the Jordan curve theorem).
    """
    sign = [-1,0,1]
    c = False
    for q in sign:
        for p in sign:
            c = c or isInsidePolygon(n,Vx,Vy,x+delta*q,y+delta*p)
    return c

def ConcaveToConvex(coordinates):
    """
    transforms a concave polygon into its convex hull.
    """
    coords = np.array(coordinates).T
    hull = ConvexHull(coords)
    plt.plot(coords[hull.vertices,0], coords[hull.vertices,1], 'b--', lw=1)
    return((coords[hull.vertices].T).tolist())
    
 
def generateFrame(xmin,xmax,ymin,ymax):
    """
    creates the frame points necessary for Vornoi diagram.
    """
    xmin,xmax,ymin,ymax  = xmin-0.2,xmax+0.2,ymin-0.2,ymax+0.2
    frame = [[xmin,y] for y in np.arange(ymin,ymax)]
    frame += [[xmax,y] for y in np.arange(ymin,ymax)]
    frame += [[x,ymin] for x in np.arange(xmin,xmax)]
    frame += [[x,ymax] for x in np.arange(xmin,xmax)]
    return(frame)
    
def generateVoronoi(points,obstacles):
    """
    creates the Voronoi diagram taking into account possible obstacles. 
    Final points are all outside every obstacle, and final edges do not 
    intersect any of them.
    """
    vor = Voronoi(points)
    simplex_valid = []
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
               simplex_valid.append(simplex.tolist())
               #pass
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
    return(vor,simplex_valid)

def dijkstra(graph, src, dst):
    """
    finds the shortest path from vertex src to vertex dst in a graph.
    """
    table = updateTable(graph,src)
    path = [dst]
    while path[-1] != src :
        path.append(table[-1][path[-1]][1] )
    path.reverse()
    return path

def updateTable(graph,src):
    """
    fills exploration table for Dijkstra's algorithm.
    """
    n = len(graph.vertices)
    table =[[False]*n]
    table[0][src] = 0,0
    visited = [src]
    while len(visited)<n :
        addLineDijkstra(table,visited,graph)
    return(table)
  
def addLineDijkstra(table,visited,graph):
    """
    updates exploration table for Dijkstra's algorithm.
    """
    # new line is last added line with some different values 
    last_line = table[-1]
    new_line = deepcopy(last_line)
    m = len(new_line)
    # summit to study
    s = visited[-1]
    # length of associated shortest path
    long = last_line[s][0]
    for j in range(m) :
        if j not in visited:
            weight = graph.edges.get((s,j),False)
            if weight : # if edge (s,j) exists
                if not(last_line[j]) : # L[j] = False
                    new_line[j] = [ long + weight, s ]
                else :
                    if long + weight < last_line[j][0] :
                        new_line[j] = [ long + weight, s ]
    table.append(new_line)
    visited.append(nextSummit(table, visited))
        
def nextSummit(table, visited):
    """
    selects the summit associated with the shortest current pathfor Dijkstra's
    algorithm.
    """
    line = table[-1]
    mini = False
    for i in range(len(line)) :
        if not(i in visited) :
            if line[i]:
                if not(mini) or line[i][0] < mini :
                    mini = line[i][0]
                    index = i
    return(index)

if __name__=='__main__':
    
    plt.figure(0,(8,4))
    plt.subplots_adjust(right=0.75)
    
#------------------------Complete with data------------------------------------

    # list of obstacles
    obstacles = [[[3,3.25,3.5,3.5,3.5,3.5,3,3,3],[0.5,0,0.5,1,1.5,2,2,1.5,1]],
                [[0,1,0],[3,4,4]],
                [[4,4.5,4,5,6],[6,5.5,5,4,7]]]
    
    # area's dimensions
    xmin,xmax,ymin,ymax = -1,8,-1,8

#------------------------------------------------------------------------------
    
    # plot obstacle before transformation 
    for obstacle in obstacles:
        plt.plot(obstacle[0]+[obstacle[0][0]],obstacle[1]+[obstacle[1][0]],"b--")
        plt.plot(obstacle[0]+[obstacle[0][0]],obstacle[1]+[obstacle[1][0]],"*b") 
    
    # transform all obstacles not convex
    for i in range(len(obstacles)):
        obstacles[i] = ConcaveToConvex(obstacles[i])
        
    # creation of all necessary points              
    points = []
    for obstacle in obstacles :
        a = deepcopy([[obstacle[0][i],obstacle[1][i]]
                        for i in range(len(obstacle[0]))])
        points += a    
    frame = generateFrame(xmin,xmax,ymin,ymax)
    points = points + frame
    
    # voronoi diagram
    vor,vertices = generateVoronoi(points,obstacles)
    
    #Graph
    valid_vertices = list(set([j for sub in vertices for j in sub]))
    G = Graph()
    G.addVertices(vor.vertices[valid_vertices[:],0],
                  vor.vertices[valid_vertices[:],1])
    for duo in vertices:
        n1,n2 = valid_vertices.index(duo[0]),valid_vertices.index(duo[1])
        G.addEdge(n1,n2,euclideanDistance(G.vertices[n1],G.vertices[n2]))
        G.addEdge(n2,n1,euclideanDistance(G.vertices[n1],G.vertices[n2]))

#-----------------Select the source and destination for the path -------------
    src = 0
    dst = 45
    path = dijkstra(G,src,dst)

#------------------------------------------------------------------------------
    
    #plot path
    for i in range(len(path)-1):
        plt.plot([G.vertices[path[i]][0],G.vertices[path[i+1]][0]],
                 [G.vertices[path[i]][1],G.vertices[path[i+1]][1]],"red")
        
    plt.plot([G.vertices[path[0]][0],G.vertices[path[1]][0]],
                 [G.vertices[path[0]][1],G.vertices[path[1]][1]],"red", 
                 label="path")
    # plot obstacle
    for obstacle in obstacles:
        plt.plot(obstacle[0]+[obstacle[0][0]],obstacle[1]+[obstacle[1][0]],
                 color="blue", label="obstacle")
        plt.plot(obstacle[0]+[obstacle[0][0]],obstacle[1]+[obstacle[1][0]],"*b") 
        
    # plot frame
    plt.plot(np.array(frame)[:,0],np.array(frame)[:,1],'*g',label="frame")
       
    plt.xlim([xmin-0.5,xmax+0.5])
    plt.ylim([ymin-0.5,ymax+0.5])
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', ncol=1,borderaxespad=0.)
    plt.title("Voronoi diagram based avoidance solution")

    
    #print(vor.vertices)
    
    #Converting the path for the map
    mapPath, factor = [],50
    for i in range(len(path)-1):
        mapPath.append(np.array([[factor*list(G.vertices[path[i]])[0]],
                                       [factor*list(G.vertices[path[i]])[1]]]))
    start = mapPath[0]
    end = mapPath[-1]

    # Constructing the autonomous sailboat
    wind_angle, wind_speed = -pi/2, 2
    
    x0= np.array([[-5,310,pi,0.2,0]]).T  #x=(x,y,θ,v,w)
    a_tw = wind_speed    # true wind force
    ψ_tw = wind_angle    # true wind angle
    r = 10      # maximale acceptable distance from target line
    ζ = pi/4    # closed hauled angle for the no-go zone
    δrmax = 1   # maximal rudder angle
    β = pi/4    # angle of the sail in crosswind 
    B = Boat(x0,a_tw, ψ_tw, r, ζ, δrmax, β)
    
    # Matplotlib parameters    
    dt = 0.1
    ax=init_figure(mapPath, factor*xmin -10, factor*xmax +10, 
                            factor*ymin -10, factor*ymax+10)
    plot_frequency = 10  # every x time period
    end_index = len(mapPath)
    
    # plotting environment
    for obstacle in obstacles:
            plt.plot([factor*x for x in obstacle[0]]+[factor*obstacle[0][0]],
                  [factor*x for x in obstacle[1]]+[factor*obstacle[1][0]],color="blue")
    plt.plot([factor*i for i in np.array(frame)[:,0]],
              [factor * i for i in np.array(frame)[:,1]],'*g',label="frame")
    
    # Runge-Kutta 45
    while True:
        plt.plot([factor*i for i in np.array(frame)[:,0]],
              [factor * i for i in np.array(frame)[:,1]],'*g',label="frame")
        for obstacle in obstacles:
            plt.plot([factor*x for x in obstacle[0]]+[factor*obstacle[0][0]],
                  [factor*x for x in obstacle[1]]+[factor*obstacle[1][0]],color="blue")
        y = solve_ivp(f_ode_45, (0,plot_frequency),(x0.T).tolist()[0],method='RK45',
                      args=(B,ax,mapPath,True))
        x0 = y.y[:,-1].reshape((5,1))
        no = np.where(np.array(mapPath).reshape(len(mapPath),2) == end.T)[0]
        new_index = extractDuplicate(no.tolist())[0]
        if new_index == 0:
            break
    
    plt.title("Path avoiddance with Voronoi diagram")
    plt.show()
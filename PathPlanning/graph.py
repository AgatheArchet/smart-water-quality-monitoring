#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 15 10:32:40 2020

@author: agathe
"""
from math import cos,sin,pi,atan2, sqrt
import matplotlib.pyplot as plt, random

class Graph:
    """
    A class to modelize an graph of various verticies.
    
    Attributes
    ----------
    vertices : organized list of tuple
        the coordinates of all vertices, with the i-th element the coordoonates
        (x,y) for the vertex number i.
    edges : dictionnary with two keys
        the key (u,v) with float value d indicates the distance d to go 
        from vextex u to v.If there is no existing distance between two 
        vertices, they are unreachables.
    path : list of integers
        the list with the optimal known path
    wind_angle : integer betwenn 0 and 2 pi rad
        current direction of the wind, with respect to unit circle convention
        north wind : 3pi rad / east wind : 0 rad / south wind : pi rad
    wind_speed : float
        current speed of the in m/s
    
    """
    def __init__(self):
        self.vertices = []
        self.edges = {}
        self.path = None
        self.wind_angle = None
        self.wind_speed = None
        
    def defineWind(self,angle,speed=5):
        """
        sets the wind characteritcs that will produce a penalty in addEdge() 
        method.
        """
        self.wind_angle = angle
        self.wind_speed = speed
        
    def addSingleVertex(self,x,y):
        """
        adds a vertex in the graph's vertices list.
        """
        self.vertices.append((x,y))
        
    def addVertices(self,list_x,list_y):
        """
        adds sevral vertices at a times.
        """
        for i in range(len(list_x)):
            self.addSingleVertex(list_x[i],list_y[i])
        
    def addEdge(self, from_node, to_node, weight):
        """
        defines the weight of edge linking "from_nod"e to "to_node" vertices, 
        and applies a wind penalty to avoid moving in the opposite direction of
        wind.
        """
        if self.wind_angle != None:
            xa,ya = self.vertices[from_node]
            xb,yb = self.vertices[to_node]
            slope = atan2(yb-ya,xb-xa)
            penalty = 0.5*self.wind_speed*cos(self.wind_angle-slope)
        else:
            penalty = 0
        self.edges[(from_node, to_node)] = weight*(1+max(0,penalty))
        #self.edges[(to_node, from_node)] = weight*(1+max(0,-penalty))
        
    def addEdgeFromCoords(self, from_node, to_node, weight):
        """
        adds edges, but directly with the two vertices' coordinates.
        """
        u = self.getAssociatedNumber(from_node[0],from_node[1])
        v = self.getAssociatedNumber(to_node[0],to_node[1])
        self.addEdge(u,v,weight)
        
    def addEdgesAuto(self):
        """
        determines automatically weight for all edges between all vertices,
        using the eclidean distance as value0
        """
        for i in range(len(self.vertices)):
            for j in range(len(self.vertices)):
                if i!=j:
                    xa,ya = self.vertices[i]
                    xb,yb = self.vertices[j]
                    dist = sqrt((yb-ya)**2+(xb-xa)**2)
                    self.addEdgeFromCoords([xa,ya],[xb,yb],dist)
                    self.addEdgeFromCoords([xb,yb],[xa,ya],dist)
    
    def getAssociatedNumber(self,x,y):
        """
        finds the index of the vertex in the graph's list, directly with its
        coordinates.
        """
        return(self.vertices.index((x,y)))
      
    def getDist(self,from_node,to_node):
        """
        gives the weight of the edge linking one vertex to another, and returns 
        an infinite number if the vertices are unreachable.
        """
        if (from_node,to_node) in self.edges.keys():
            value = self.edges[(from_node,to_node)]
        else:
            value = float('inf')
        return(value)
        
    def getDistFromCoords(self,from_node,to_node):
        """
        calls getDist() method knowing the two vertices' coordinates
        """
        u = self.getAssociatedNumber(from_node[0],from_node[1])
        v = self.getAssociatedNumber(to_node[0],to_node[1])
        return(self.getDist(u,v))
        
    def getPathLength(self, path):
        """
        calculs the current path lenght
        """
        value = 0
        for i in range(len(self.vertices)-1):
            value += self.getDist(path[i],path[i+1])
        value += self.getDist(path[-1],path[0])
        return(value)
         
    def plot(self):
        """
        gives a graphical representation of the path.
        """
        x,y = [],[]
        for points in self.path:
            x.append(self.vertices[points][0])
            y.append(self.vertices[points][1])
        x.append(self.vertices[self.path[0]][0])
        y.append(self.vertices[self.path[0]][1])
        plt.plot(x,y, 'xg-')
        
    def solveRandom(self,nb_of_tour=1000):
        """
        solver based on a random strategy.
        """
        n = len(self.vertices)
        self.path = random.sample(range(n),n)
        length = self.getPathLength(self.path)
        for t in range(0,nb_of_tour):
            print("length : "+str(length))
            i,j = sorted(random.sample(range(0,n),2)); #diferent random points
            newPath =  self.path[:i]+self.path[j:j+1]+self.path[i+1:j]+ self.path[i:i+1]+self.path[j+1:];
            if self.getPathLength(newPath) < length:
                self.path = newPath
                length = self.getPathLength(newPath)
                
                
        
if __name__=='__main__':
    
    G = Graph()
    G.defineWind(pi/2)
    G.addVertices([0,1,3,2.5,6,5,4],[0,1,6,1,0.5,4,1])
    
#    G.addEdgeFromCoords((0,0),(1,1),8)
#    G.addEdgeFromCoords((1,1),(3,6),2)
#    G.addEdgeFromCoords((3,6),(0,0),5)
    G.addEdgesAuto()
    G.solveRandom()
    G.plot()
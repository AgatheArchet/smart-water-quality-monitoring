#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 15 10:32:40 2020

@author: agathe
"""
from math import cos,sin,pi,atan2, sqrt
import matplotlib.pyplot as plt
import numpy as np
import random
from scipy.spatial import Delaunay
from copy import deepcopy
import time

class Graph:
    """
    A class to modelize an asymetrical directed graph of various verticies.
    
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
        north wind : pi/2 rad / west wind : 0 rad / south wind : 3pi/2 rad.
    wind_speed : float
        current speed of the in m/s.
    solver : string
        the algorithm used to find the optimal path.
    time_solved : float
        the total time needed to find the optimal path.
    path_evolution : list of float
        all steps of the path optimization, for graphical representation.
    time_evolution : list of float
        all steps of the time each time the path's length is reduced, for 
        graphical representation.
    
    """
    def __init__(self):
        self.vertices = []
        self.edges = {}
        self.path = None
        self.wind_angle = None
        self.wind_speed = None
        self.solver = None
        self.time_solved = None
        self.path_evolution = []
        self.time_evolution = []
        
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
        defines the weight of edge linking "from_node" to "to_node" vertices, 
        and applies a wind penalty to avoid moving in the opposite direction of
        wind.
        """
        if self.wind_angle != None:
            xa,ya = self.vertices[from_node]
            xb,yb = self.vertices[to_node]
            slope = atan2(yb-ya,xb-xa)
            penalty = self.wind_speed*cos(self.wind_angle-slope)
        else:
            penalty = 0
        self.edges[(from_node, to_node)] = weight*(1+max(0,penalty))
        #self.edges[(to_node, from_node)] = weight*(1+max(0,-penalty))
        
    def addEdgeFromCoords(self, from_node, to_node, weight=0):
        """
        adds edges, but directly with the two vertices' coordinates.
        """
        u = self.getAssociatedNumber(from_node[0],from_node[1])
        v = self.getAssociatedNumber(to_node[0],to_node[1])
        if weight==0:
            weight = sqrt((to_node[1]-from_node[1])**2+(to_node[0]-from_node[0])**2)
        self.addEdge(u,v,weight)
        self.addEdge(v,u,weight)
        
    def addEdgesAll(self):
        """
        determines automatically weight for all edges between all vertices,
        using the eclidean distance as value.
        """
        for i in range(len(self.vertices)):
            for j in range(len(self.vertices)):
                if i!=j:
                    xa,ya = self.vertices[i]
                    xb,yb = self.vertices[j]
                    dist = sqrt((yb-ya)**2+(xb-xa)**2)
                    self.addEdgeFromCoords([xa,ya],[xb,yb],dist)
                    self.addEdgeFromCoords([xb,yb],[xa,ya],dist)
                    
    def addEdgesDelaunay(self):
        """
        determines automatically weight for edges defined by the Delaunay's 
        triangulation.
        """
        x = [k[0] for k in self.vertices]
        y = [k[1] for k in self.vertices]
        points = np.array([x,y]).T
        tri = Delaunay(points)
        plt.triplot(points[:,0], points[:,1],tri.simplices.copy(),c='#BBBBBB')
        triangles = points[tri.simplices]
        for summit in triangles :
            self.addEdgeFromCoords(summit[0].tolist(),summit[1].tolist())
            self.addEdgeFromCoords(summit[1].tolist(),summit[2].tolist())
            self.addEdgeFromCoords(summit[2].tolist(),summit[0].tolist())
            
    def addObstacleatCoords(self,x,y,radius):
        """
        changes to "infinte" value of all edges passing through the obstacle.
        """
        #TODO : add a condition 
    
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
        # Exception : back to the begining
        if self.getDist(path[-1],path[0]) == float('inf'):
            x0,y0 = self.vertices[path[0]]
            xEnd,yEnd = self.vertices[path[-1]]
            value += sqrt((x0-xEnd)**2+(y0-yEnd)**2)
        else:
            value += self.getDist(path[-1],path[0])
        return(value)
         
    def plot(self,gradual=False):
        """
        gives a graphical representation of the path.
        """
        plt.figure(figsize=(12,4))
        if len(self.path_evolution)!=0:
            plt.subplot(121)
        x,y = [],[]
        for points in self.path:
            x.append(self.vertices[points][0])
            y.append(self.vertices[points][1])
        x.append(self.vertices[self.path[0]][0])
        y.append(self.vertices[self.path[0]][1])
        max_X,max_Y,min_X,min_Y = max(x),max(y),min(x),min(y)
        plt.xlim([min_X-0.2*(max_X-min_X),max_X+0.4*((max_X-min_X))])
        plt.ylim([min_Y-0.2*(max_Y-min_Y),max_Y+0.4*(max_Y-min_Y)])
        # points and colors
        if not(gradual):
            plt.plot(x,y, 'xg-')
        else:
            plt.plot(x[0],y[0],'go',markersize = 10)
            for i in range(len(x)-1):
                plt.plot([x[i],x[i+1]],[y[i],y[i+1]], 'x-',color=(i/len(x),(1-i/len(x)),0))
        # wind arrow
        if self.wind_angle != None:
            self.draw_arrow(max_X+0.1*max_X,max_Y+0.1*max_Y,self.wind_angle,0.03*(max_X-min_X),self.wind_speed,'blue')
        plt.title('Grah solved by {} in {:.1f} sec : path of {:.2f}'.format(self.solver,self.time_solved,self.getPathLength(self.path)))
        plt.xlabel("X")
        plt.ylabel("Y")
        if len(self.path_evolution)!=0:
            plt.subplot(122)
            plt.plot(self.time_evolution,self.path_evolution)
            plt.title("Evolution of solution as function of time")
            plt.xlabel("time (sec)")
            plt.ylabel("length")
        plt.show()
        
    def draw_arrow(self,x,y,θ,e,w,col):
        """
        symbolizes the wind angle and speed on the graphical representation by 
        an oriented arrow of variable width.
        """
        M1=np.array([[0,6*e,5*e,6*e,5*e],[0,0,-e,0,e]])
        M=np.append(M1,[[1,1,1,1,1]],axis=0)
        R=np.array([[cos(θ),-sin(θ),x],[sin(θ),cos(θ),y],[0,0,1]])
        plt.plot((R@M)[0, :], (R@M)[1, :], col, linewidth = w/2)
        plt.text(x+2*e, y+2*e, "wind", color="blue")
        
    def solveRandom(self,nb_of_tour=1000, show_evolution = False):
        """
        solver based on a random strategy. The graph is assumed to have all its 
        vertices linked one to each other (so it is not necessarily a complete 
        graph).
        """
        self.solver = "Random"
        timer = time.time()
        n = len(self.vertices)
        self.path = random.sample(range(n),n)
        length = self.getPathLength(self.path)
        for t in range(0,nb_of_tour):
            print(str(int(100*t/nb_of_tour))+"% length : "+str(length))
            #switch 2 random different vertices to create a new path
            i,j = sorted(random.sample(range(0,n),2))
            newPath =  self.path[:i]+self.path[j:j+1]+self.path[i+1:j]+ self.path[i:i+1]+self.path[j+1:]
            #test of improvement
            if self.getPathLength(newPath) < length:
                self.path = newPath
                length = self.getPathLength(newPath)
                if(show_evolution):
                    self.path_evolution.append(length)
                    self.time_evolution.append(time.time()-timer)
        if(show_evolution):
            self.path_evolution.append(length)
            self.time_evolution.append(time.time()-timer)
        self.time_solved = time.time()-timer
                
    def solveLoop(self,nb_of_tour=10, show_evolution=False):
        """
        solver based on a loop strategy. The graph is assumed to have all its 
        vertices linked one to each other (so it is not necessarily a complete 
        graph).
        """
        self.solver = "Loop"
        timer = time.time()
        n = len(self.vertices)
        self.path = random.sample(range(n),n)
        length = self.getPathLength(self.path)
        for t in range(0,nb_of_tour):
            for j in range(0,n):
                for i in range(0,j-1):
                    print(str(int(100*t/nb_of_tour))+"% length : "+str(length))
                    #switch 2 different vertices to create a new path
                    newPath =  self.path[:i]+self.path[j:j+1]+self.path[i+1:j]+ self.path[i:i+1]+self.path[j+1:];
                    #test of improvement
                    if self.getPathLength(newPath) < length:
                        self.path = newPath
                        length = self.getPathLength(newPath)
                        if(show_evolution):
                            self.path_evolution.append(length)
                            self.time_evolution.append(time.time()-timer)
        if(show_evolution):
            self.path_evolution.append(length)
            self.time_evolution.append(time.time()-timer)
        self.time_solved = time.time()-timer 

    def solveNearestNeighbour(self):
        """
        solver based on a nearest neighbour strategy, assuming edges are defined
        accordingly to the Delaunay's triangulation. In that, the graph is 
        strongly connected (every vertex is reachable from every other vertex).
        """
        self.solver = "Nearest Neighbour"
        timer = time.time()
        unvisited_vertices = [k for k in range(1,len(self.vertices))]
        visited_vertices = [0]
        while len(unvisited_vertices)!=0:
            print(str(int(100*len(visited_vertices)/len(self.vertices)))+" %")
            min_vertex = visited_vertices[-1]
            vertex = visited_vertices[-1]
            for neighbour in unvisited_vertices:
                if self.getDist(vertex,neighbour) < self.getDist(vertex,min_vertex):
                    min_vertex = neighbour
            visited_vertices.append(min_vertex)
            unvisited_vertices.remove(min_vertex)
        self.path = visited_vertices
        self.time_solved = time.time()-timer
        
    def solveGenetic(self, temperature=100000, pop_size=20, show_evolution = False):
        """
        solver based on a genetic strategy. The graph is assumed to have all its 
        vertices linked one to each other (so it is not necessarily a complete 
        graph).
        """
        self.solver = "Genetic"
        timer = time.time()
        generation = 0
        pop_size = round(pop_size/2)*2
        population, fitness = [],[]
        n = len(self.vertices)
        #initialiaze population
        for k in range(pop_size):
            population.append(tuple(random.sample(range(n),n)))
        fitness = [self.getPathLength(p) for p in population]    
        while temperature > 10:
            #parent selection
            newPop = []
            sorted_by_fitness = sorted(zip(fitness, population))
            population = 2*[element for _, element in sorted_by_fitness][0:int(pop_size/2)]
            fitness.sort()
            fitness = 2*fitness[0:int(pop_size/2)]
            if (show_evolution):
                if ((generation%10)==0):
                    self.path_evolution.append(fitness[0])
                    self.time_evolution.append(time.time()-timer)
            # mutation
            for k in range(pop_size):
                path = deepcopy(population[k])
                while(True):
                    i,j = sorted(random.sample(range(0,n),2))
                    newPath = path[:i]+path[j:j+1]+path[i+1:j]+path[i:i+1]+path[j+1:]
                    if self.getPathLength(newPath)<=fitness[k]:
                        newPop.append(newPath)
                        break
                    else:
                        # accept the rejected children with a probability of 0.98
                        prob = 2.7**(-(self.getPathLength(newPath)-fitness[k])/max(fitness))
                        if prob>0.987:
                            newPop.append(newPath)
                            break
            temperature = 0.99*temperature
            generation += 1
            population = newPop
            fitness = [self.getPathLength(p) for p in population]
            print("Time : "+str(int(temperature))+"  Generation: "+str(generation))
        sorted_by_fitness = sorted(zip(fitness, population))
        self.path = [element for _, element in sorted_by_fitness][0]
        if(show_evolution):
            self.path_evolution.append(self.getPathLength(self.path))
            self.time_evolution.append(time.time()-timer)
        self.time_solved = time.time()-timer
        
        
      
        
if __name__=='__main__':
    
    G = Graph()
    G.defineWind(angle=pi/2)
    x,y = [0,1,3,2.5,6,5,4],[0,1,6,1,0.5,4,1]

    G.addVertices(x,y)

    G.addEdgesAll()
    G.changeNodeIntoObstacle(5)
    G.solveRandom(100000,show_evolution=True)
    
#    G.addEdgesAll()
#    G.solveLoop(100,show_evolution=True)
    
#    G.addEdgesDelaunay()
#    G.solveNearestNeighbour()
    
#    G.addEdgesAll()
#    G.solveGenetic(temperature = 1000000, pop_size = 20, show_evolution=True)
    
    G.plot(gradual=True)
    
    #TODO : modify algorithms so it works with circle areas
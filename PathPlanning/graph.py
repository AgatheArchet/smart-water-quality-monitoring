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

from area_generator import *

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
        self.edges_obstacle = {}
        self.path = None
        self.subpath_obstacle = {}
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
        
    def addEdge(self, from_node, to_node, weight, obstacle=False):
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
        if not(obstacle):
            self.edges[(from_node, to_node)] = weight*(1+max(0,penalty))
        else:
            self.edges_obstacle[(from_node, to_node)] = weight*(1+max(0,penalty))
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
                    dist = euclideanDistance((xa,ya),(xb,yb))
                    self.addEdgeFromCoords([xa,ya],[xb,yb],dist)
                    self.addEdgeFromCoords([xb,yb],[xa,ya],dist)
                    
    def addEdgesDelaunay(self):
        """
        determines automatically weight for edges defined by a Delaunay
        triangulation.
        """
        plt.figure(1)
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
        plt.title("Delaunay triangulation for the associated map")
            
#    def addObstacleAtCoords(self,x,y,r):
#        """
#        changes to "infinte" value of all edges passing through the obstacle.
#        E is the starting point of the ray => (x1,y1)
#        L is the end point of the ray => (x2,y2)
#        C is the center of the circle => (x,y)
#        r is the radius of the circle
#        The intersection is found by the parametric equations:
#        Px = Ex + tdx      and      Py = Ey + tdy
#        plugged into the circle equation :
#        (x - h)2 + (y - k)2 = r2,  with (h,k) = center of circle.
#        This leads to solving the following quadratic equation :
#        t2*( d DOT d ) + 2t*( f DOT d ) + ( f DOT f - r2 ) = 0
#        """
#        for key in G.edges.keys():
#            x1,y1 = self.vertices[key[0]]
#            x2,y2 = self.vertices[key[1]]
#            # Direction vector of ray, from start to end
#            d = np.array([x2-x1,y2-y1]) 
#            # Vector from center sphere to ray start 
#            f = np.array([x1-x,y1-y])
#            
#            a = d@(d.T)
#            b = 2*f@(d.T)
#            c = f@(f.T) - r*r
#            discriminant = b*b -4*a*c
#            if (discriminant<0):
#                print("no intersection")
#            else :
#                discriminant = sqrt(discriminant)
#                t1 = (-b - discriminant)/(2*a);
#                t2 = (-b + discriminant)/(2*a);
#                print("ok   ("+str(x1)+","+str(y1)+")  ("+str(x2)+","+str(y2)+") + t1: "+str(t1)+"  t2: "+str(t2))
#                if ((t1>1 and t2>1) or (t1<0 and t2<0)) : # the circle does not touch the circle
#                    pass
#                elif (t1<0 and t2>1): # the segment is completely inside the circle
#                    print(key)
#                    self.edges[key] = float('inf')
#                else:
#                    inter2_x = x1 + t2*d[0] 
#                    inter2_y = y1 + t2*d[1] 
#                    inter1_x = x1 + t1*d[0] 
#                    inter1_y = y1 + t1*d[1]
#                    print(key)
#                    self.edges[key] = float('inf')
#                    if t1<0 : # one intersection with the segment
#                        print(inter2_x,inter2_y)
#                        #return (inter2_x,inter2_y)
#                        plt.plot(inter2_x,inter2_y)
#                    if t2>1:  # one intersection with the segment
#                        print(inter1_x,inter1_y)
#                        #return(inter1_x,inter1_y)
#                        plt.plot(inter1_x,inter1_y)
#                    else:   # two intersections with the segment
#                        print(inter1_x,inter1_y,inter2_x,inter2_y)
#                        #return(inter1_x,inter1_y,inter2_w,inter2_y)
                        
    def addPolygoneObstacleAtCoords(self,list_x,list_y,safety_distance = 0.2):
        """
        proposes an alternative path if one of the vertex touches the obstacle.
        The obstacle is assumed to be convex, and to have no vertex inside it.
        """
        plt.figure(0)
        distance  = sqrt(2*(safety_distance**2))
        new_poly = []
        n = len(list_x)
        # polygone buffering with a safety distance
        for i in range(n):
            xa, ya = list_x[i],list_y[i]
            vect1= np.array([list_x[i-1]-list_x[i],list_y[i-1]-list_y[i]])
            vect2= np.array([list_x[(i+1)%n]-list_x[i],list_y[(i+1)%n]-list_y[i]])
            bissector = np.linalg.norm(vect1)*vect2+np.linalg.norm(vect2)*vect1
            middle_angle = atan2(bissector[1],bissector[0])+pi
            #bissector_angle = -(middle_angle-atan2(vect1[1],vect1[0])-pi)%(2*pi)
            #distance = sqrt(2*safety_distance**2*(1-cos(pi-2*bissector_angle)))
            xnew = xa+distance*cos(middle_angle)
            ynew = ya+distance*sin(middle_angle)
            new_poly.append((xnew,ynew))
        plt.plot(list_x + [list_x[0]], list_y + [list_y[0]], color ="black")
        plt.plot([p[0] for p in new_poly]+[new_poly[0][0]],[p[1] for p in new_poly]+[new_poly[0][1]])
        # remove vertices that are inside the inflated polygon
        #TODO
        # remove from dictionnary edges that intersect with the obstacle
        keysToTransform = []
        for key in self.edges.keys():
            x1,y1 = self.vertices[key[0]]
            x2,y2 = self.vertices[key[1]]
            for i in range(n+1):
                if doIntersect((x1,y1),(x2,y2),new_poly[i%n],new_poly[(i+1)%n]):
                    keysToTransform.append(key)
        keysToTransform = list(set(keysToTransform))
        for k in keysToTransform:
            self.edges.pop(k, None)
            length,path_poly = self.findAlternativePath(k,new_poly)
            self.addEdge(k[0], k[1], length, obstacle=True)
            self.subpath_obstacle[(k[0], k[1])] = [(new_poly[p][0],new_poly[p][1]) for p in path_poly]
            
    def findAlternativePath(self,key,polyCoords):
        """
        determines the shortest distance between two given point in key, 
        bypassing the obstacle with a safety distance from it.
        """
        x1,y1 = self.vertices[key[0]]
        x2,y2 = self.vertices[key[1]]
        vertex1_altern,vertex2_altern = [],[]
        proj1, proj2 = 0,0
        link1, link2 = None,None
        n = len(polyCoords)
        # findind all accessible vertices of the polygon (without passing through it)
        for point in range(n):
            intersection1, intersection2 = False, False
            for point2 in [i for i in range(n) if (i!=point and i!=(point-1)%n)]:
                if (doIntersect((x1,y1),polyCoords[point],polyCoords[point2],polyCoords[(point2+1)%(n)])):      
                    intersection1 = True
                if (doIntersect((x2,y2),polyCoords[point],polyCoords[point2],polyCoords[(point2+1)%(n)])):      
                    intersection2 = True
            if not(intersection1):
#                plt.plot([x1,polyCoords[point][0]],[y1,polyCoords[point][1]],'--',color="green")
                vertex1_altern.append(point)
            if not(intersection2):
#                plt.plot([x2,polyCoords[point][0]],[y2,polyCoords[point][1]],'.-',color="orange")
                vertex2_altern.append(point)
        # comparing legnth of projection of each vertex on ((x1,y1),(x2,y2)) line.
        for point in vertex1_altern:
            angle = atan2(polyCoords[point][1]-y1,polyCoords[point][0]-x1)
            dist =  euclideanDistance(polyCoords[point],(x1,y1))
            projection = abs(dist*cos(angle))
            if projection > proj1:
                proj1 = projection
                link1 = point
        for point in vertex2_altern:
            angle = atan2(polyCoords[point][1]-y2,polyCoords[point][0]-x2)%pi
            dist = euclideanDistance(polyCoords[point],(x2,y2))
            projection = abs(dist*cos(angle))
            if projection > proj2:
                proj2 = projection
                link2 = point
        if link1 == link2:
            #add distance on edge
#            plt.plot((x1,polyCoords[link1][0]),(y1,polyCoords[link1][1]),"red")
#            plt.plot((x2,polyCoords[link2][0]),(y2,polyCoords[link2][1]),"red")
            dist = euclideanDistance(polyCoords[link1],(x1,y1)) + euclideanDistance(polyCoords[link1],(x2,y2))
            return(dist,[link1])
        else:
            # selecting best path on polygon passing by link1
            length_other_path1 = float('inf')
            other_path1 = []
            for point in vertex2_altern :
                pathCW, pathCCW = getSliceSequences(link1,point,n)
                lcw = sum([euclideanDistance(polyCoords[pathCW[i]],polyCoords[pathCW[i+1]]) for i in range(len(pathCW)-1)])
                lcw += euclideanDistance((x1,y1),polyCoords[pathCW[0]])
                lcw += euclideanDistance((x2,y2),polyCoords[pathCW[-1]])
                lccw = sum([euclideanDistance(polyCoords[pathCCW[i]],polyCoords[pathCCW[i+1]]) for i in range(len(pathCCW)-1)])
                lccw += euclideanDistance((x1,y1),polyCoords[pathCCW[0]])
                lccw += euclideanDistance((x2,y2),polyCoords[pathCCW[-1]])
                if lcw<length_other_path1 or lccw<length_other_path1:
                    if lcw<lccw:
                        length_other_path1 = lcw
                        other_path1 = deepcopy(pathCW)
                    else:
                        length_other_path1 = lccw
                        other_path1 = deepcopy(pathCCW)
            # selecting best path on polygon passing by link2
            length_other_path2 = float('inf')
            other_path2 = []
            for point in vertex1_altern :
                pathCW, pathCCW = getSliceSequences(link2,point,n)
                lcw = sum([euclideanDistance(polyCoords[pathCW[i]],polyCoords[pathCW[i+1]]) for i in range(len(pathCW)-1)])
                lcw += euclideanDistance((x2,y2),polyCoords[pathCW[0]])
                lcw += euclideanDistance((x1,y1),polyCoords[pathCW[-1]])
                lccw = sum([euclideanDistance(polyCoords[pathCCW[i]],polyCoords[pathCCW[i+1]]) for i in range(len(pathCCW)-1)])
                lccw += euclideanDistance((x2,y2),polyCoords[pathCCW[0]])
                lccw += euclideanDistance((x1,y1),polyCoords[pathCCW[-1]])
                if lcw<length_other_path2 or lccw<length_other_path2:
                    if lcw<lccw:
                        length_other_path2 = lcw
                        other_path2 = deepcopy(pathCW)
                    else:
                        length_other_path2 = lccw
                        other_path2 = deepcopy(pathCCW)
            #selecting the shortest path on polygon
            if length_other_path1 <= length_other_path2:
#                plt.plot((x1,polyCoords[link1][0]),(y1,polyCoords[link1][1]),"red")
#                plt.plot([polyCoords[p][0] for p in other_path1],[polyCoords[p][1] for p in other_path1],"red" )
#                plt.plot((polyCoords[other_path1[-1]][0],x2),(polyCoords[other_path1[-1]][1],y2),"red")
                return(length_other_path1,other_path1[::-1])
            else:
#                plt.plot((x2,polyCoords[link2][0]),(y2,polyCoords[link2][1]),"red")
#                plt.plot([polyCoords[p][0] for p in other_path2],[polyCoords[p][1] for p in other_path2],"red" )
#                plt.plot((polyCoords[other_path2[-1]][0],x1),(polyCoords[other_path2[-1]][1],y1),"red")
                return(length_other_path2,other_path2)
                
    def addObstacleIntoFinalPath(self):
        n = len(self.path)
        points = [(self.path[i],self.path[(i+1)%n]) for i in range(n)]
        for i in range(len(points)):
            if points[i] in self.edges.keys():
                pass
            elif points[i] in self.subpath_obstacle.keys():
                nk = len(self.subpath_obstacle[points[i]])
                for k in range(nk):
                    self.path.insert(i+1+k, self.subpath_obstacle[points[i]][nk-k-1])
            
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
        elif (from_node,to_node) in self.edges_obstacle.keys():
            value = self.edges_obstacle[(from_node,to_node)]
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
            try :
                xEnd,yEnd = self.vertices[path[-1]]
            except:
                xEnd,yEnd = path[-1]
            value += euclideanDistance((x0,y0),(xEnd,yEnd))
        else:
            value += self.getDist(path[-1],path[0])
            
        return(value)
        
    def plot(self):
        plt.figure(0,figsize=(12,4))
        for key in self.edges.keys():
            x1,y1 = self.vertices[key[0]]
            x2,y2 = self.vertices[key[1]]
            plt.plot([x1,x2],[y1,y2],linestyle='dotted',color="lightgrey")
            plt.plot(x1,y1,marker='o',color="cyan")
            plt.plot(x2,y2,marker='o',color="cyan")
        for key in self.edges_obstacle.keys():
            x1,y1 = self.vertices[key[0]]
            x2,y2 = self.vertices[key[1]]
            for i in range(0,len(self.subpath_obstacle[(key[0],key[1])])):
                p1 = self.subpath_obstacle[(key[0],key[1])][i]
                p2 = self.subpath_obstacle[(key[0],key[1])][(i+1)%len(self.subpath_obstacle[(key[0],key[1])])]
                plt.plot(p1[0],p1[1],marker='o',color="pink")
                if i!=len(self.subpath_obstacle[(key[0],key[1])]):
                    plt.plot([p1[0],p2[0]],[p1[1],p2[1]],linestyle='dotted',color="salmon")
            plt.plot((x1,self.subpath_obstacle[(key[0],key[1])][i][0]),(y1,self.subpath_obstacle[(key[0],key[1])][i][1]),linestyle='dotted',color="salmon")
        plt.title("Graph before resolution")
            
         
    def plotPath(self,gradual=False, obstacle_x=[], obstacle_y=[]):
        """
        gives a graphical representation of the path.
        """
        plt.figure(1,figsize=(12,4))
        if len(self.edges_obstacle)>0 :
            self.addObstacleIntoFinalPath()
            plt.plot(obstacle_x+[obstacle_x[0]],obstacle_y+[obstacle_y[0]],"blue")
        if len(self.path_evolution)!=0:
            plt.subplot(121)
        x,y = [],[]
        for points in self.path:
            if isinstance(points, tuple):
                x.append(points[0])
                y.append(points[1])
            else :
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
        unvisited_vertices = list(range(1,len(self.vertices)))
        visited_vertices = [0]
        while len(unvisited_vertices)!=0:
            #print(str(int(100*len(visited_vertices)/len(self.vertices)))+" %")
            min_vertex = visited_vertices[-1]
            vertex = visited_vertices[-1]
            for neighbour in unvisited_vertices:
                if self.getDist(vertex,neighbour) <= self.getDist(vertex,min_vertex):
                    min_vertex = neighbour
            visited_vertices.append(min_vertex)
            unvisited_vertices.remove(min_vertex)
        if abs(atan2(self.vertices[visited_vertices[-1]][1],self.vertices[visited_vertices[-1]][0]) -self.wind_angle)<pi/12:
            self.path = visited_vertices[::-1]
        else:
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
        for k in range(int(pop_size)):
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
            for k in range(int(pop_size)):
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
        
def onSegment(p, q, r): 
    if ( (q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and 
           (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))): 
        return True
    return False
        
def orientation(p, q, r): 
    """
    finds the orientation of an ordered triplet ((px,py),(qx,qy),(rx,ry) 
    Returned values: 
     0 : Colinear points, 1 : Clockwise points, 2 : Counterclockwise 
    """  
    val = (float(q[1]-p[1])*(r[0]-q[0]))-(float(q[0]-p[0])*(r[1]-q[1])) 
    if (val > 0): 
        return 1 # Clockwise orientation
    elif (val < 0): 
        return 2 # Counterclockwise orientation 
    else: 
        return 0  # Colinear orientation    
    
def doIntersect(p1,q1,p2,q2): 
    """
    determines if line1 (p1,q1) and line2(p2,q2) intersect.
    Each point is at format : p1 = (px1,py1)
    """  
    o1 = orientation(p1, q1, p2) 
    o2 = orientation(p1, q1, q2) 
    o3 = orientation(p2, q2, p1) 
    o4 = orientation(p2, q2, q1) 
    # General case 
    if ((o1 != o2) and (o3 != o4)): 
        return True
    # Special Cases 
    # p1 , q1 and p2 are colinear and p2 lies on segment p1q1 
    if ((o1 == 0) and onSegment(p1, p2, q1)): 
        return True
    # p1 , q1 and q2 are colinear and q2 lies on segment p1q1 
    if ((o2 == 0) and onSegment(p1, q2, q1)): 
        return True
    # p2 , q2 and p1 are colinear and p1 lies on segment p2q2 
    if ((o3 == 0) and onSegment(p2, p1, q2)): 
        return True
    # p2 , q2 and q1 are colinear and q1 lies on segment p2q2 
    if ((o4 == 0) and onSegment(p2, q1, q2)): 
        return True
    return False

def isClockwise(coords):
    """
    detects in which sens a convex polygon coords are ordered (clockwise or
    counter-clockwise).
    """
    res = 0
    for i in range(len(coords)):
        res += (coords[(i+1)%len(coords)][0]-coords[i][0])*(coords[(i+1)%len(
                coords)][1]+coords[i][1])
    return(res>0)
    
def getSliceSequences(i,j,n):
    """
    returns 2 slieing sequences (from left to right and right to left) for two 
    indices of a n-size list. The two resulting sequences begin by i and end by 
    j.
    """
    liste = list(range(n))
    maxi,mini = max(i,j), min(i,j)
    list1 = liste[mini:maxi+1]
    list2 = (liste[maxi:] + liste[:mini+1])
    if list1[0]!=i:
        list1 = list1[::-1]
    if list2[0]!=i:
        list2 = list2[::-1]
    return(list1,list2)

def euclideanDistance(point1,point2):
    l = sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
    return(l)
      
def isInsidePolygon(n,Vx,Vy,x,y):
    """
    Verifies is a given point at coords (x,y) is inside a given polygon (Vx,Vy)
    of n summits (based upon the Jordan curve theorem).
    """
    c = False
    for i in range(0,n):
        j = (i-1)%n
        if (((Vy[i]>y)!=(Vy[j]>y)) and 
            (x < (Vx[j]-Vx[i])*(y-Vy[i])/(Vy[j]-Vy[i])+Vx[i])):
            c = not(c)
    return c 
        
if __name__=='__main__':
    
    G = Graph()
    G.defineWind(angle=pi/2)
    
    #square
   # x,y = [0,1,3,2.5,6,5,4],[0,1,6,1,0.5,4,1]
    
    #Grid
    nb_of_points = 100
    GPSpoints = np.array([[0,0],[0,100],[100,100],[100,0],[0,0]])
    A = Area(nb_of_points,GPSpoints[0,:],GPSpoints,"grid")
    A.placeMeasurementPoints()
    x = A.points_lat.reshape(nb_of_points).tolist()
    y = A.points_lon.reshape(nb_of_points).tolist()
    
    #round
#    center, beginning = [1,1],[4,4]
#    C = Area(65,beginning,beginning,"circle",center, angle_division=16)
#    C.placeMeasurementPoints()
#    x,y = C.points_lat, C.points_lon

    G.addVertices(x,y)


    #G.addEdgesAll()
    G.addEdgesDelaunay()

    #obx,oby = [3,3.25,3.5,3.5,3],[0.5,0,0.5,2,2]
    obx,oby = [2.5,7.5,7.5,5,2.5],[0,0,15,17,15]
    G.addPolygoneObstacleAtCoords(obx,oby,safety_distance = 1.5)
    

#    G.solveRandom(100000,show_evolution=True)
#    G.solveLoop(100,show_evolution=True)
    G.solveNearestNeighbour()
#    G.solveGenetic(temperature = 1000000, pop_size = 20, show_evolution=True)
    
    
    G.plot()
    G.plotPath(gradual=True,obstacle_x = obx, obstacle_y = oby)
    plt.figure(1)
    plt.show()

    #TODO : modify algorithms so it works with circle areas
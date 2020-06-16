#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 11 16:10:42 2020

@author: agathe
"""
# Geographical zone used : Plymouth
# http://www.dmap.co.uk/utmworld.htm
#utm.to_latlon(418666.8701892403, 5581094.969314943, 30, 'U') #UTMXY to swg84

import utm
import math as m, numpy as np
from area_generator import Area
from graph import Graph
from scipy.spatial import Delaunay

def readData(filename):
    lat, lon = [],[] 
    with open(filename, 'r') as f:
        if f.readline()=='#beginning\n':
            start = (f.readline()).replace(","," ").replace(";"," ").replace("\n","").split()
            beginning = [float(start[0]),float(start[1])]
            f.readline()
        for line in f:
            points = line.replace(","," ").replace(";"," ").replace("\n","").split()
            lat.append(float(points[0]))
            lon.append(float(points[1]))
    f.close()
    return(beginning,lat,lon)

def LatLonToUTMXY(list_lat,list_lon):
    x,y = [],[]
    for i in range(len(list_lat)):
        res = utm.from_latlon(list_lat[i], list_lon[i]) # wgs84 to UTMXY
        x.append(res[0])
        y.append(res[1])
    return(x,y)
    
def normalize(list_x,list_y):
    max_X,min_X = max(list_x), min(list_x)
    max_Y,min_Y = max(list_y), min(list_y)
    list_x_norm = [(x-min_X)/(max_X-min_X) for x in list_x]
    list_y_norm = [(y-min_Y)/(max_Y-min_Y) for y in list_y]
    return(list_x_norm,list_y_norm)
    
def euclideanDistance(xa,xb,ya,yb):
    return(m.sqrt((yb-ya)**2+(xb-xa)**2))
      
if __name__=='__main__':
    
    # Genarating points, optional if all points are already known
    GPSpoints = np.array([[50.352473,-4.161624],[50.352473,-4.134152],[50.343769,-4.134152],[50.343769,-4.161624],[50.352473,-4.161624]])
    A = Area(100,GPSpoints[0,:],GPSpoints,"square")
    A.placeMeasurementPoints()
    A.generateFile()
    
    # Reading points from the file
    begining,lat,lon = readData("points.txt")
    x_list,y_list = LatLonToUTMXY(lat,lon)
    x_list,y_list = normalize(x_list,y_list)
    
    # Creating a graph
    G = Graph()
    G.addVertices(x_list,y_list)
    G.defineWind(m.pi/2,5)
    
    # Random strategy
#    G.addEdgesAuto()
#    G.solveRandom(100000)
#    G.plot(gradual=True)
    
    # Loop strategy
#    G.addEdgesAuto()
#    G.solveLoop(20)
#    G.plot(gradual=True)

    #Delaunay Strategy
    points = np.array([x_list,y_list]).T
    tri = Delaunay(points)
    plt.triplot(points[:,0], points[:,1], tri.simplices.copy())
    triangles = points[tri.simplices]
    for summit in triangles :
        G.addEdgeFromCoords(summit[0],summit[1])
        G.addEdgeFromCoords(summit[1],summit[2])
        G.addEdgeFromCoords(summit[2],summit[0])
    #To do
    G.plot(gradual=True)
#----------------------------------------------------------

# Added constraints : wind direction

#First approach : Delaunay triangulation to limit possible neighbours
#                 nearest neighbour strategy

#Second approch : random path at begining
#                  generate a new path by switching two cities
#                  select the best path between the two
#                   do it again
   
# Third approach : Bionic tour strategy     
#
# Fourth approach : genetic algorithms ?           




#TO DO Delaunay ? with https://github.com/jmespadero/pyDelaunay2D
#tri = Delaunay(points)
#plt.triplot(points[:,0], points[:,1], tri.simplices.copy())


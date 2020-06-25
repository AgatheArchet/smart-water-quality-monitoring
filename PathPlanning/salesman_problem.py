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
import matplotlib.pyplot as plt
from boat import *


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
    x_list_utm,y_list_utm = LatLonToUTMXY(lat,lon)
    x_list,y_list = normalize(x_list_utm,y_list_utm)
    
    # Creating a graph
    G = Graph()
    G.addVertices(x_list,y_list)
    wind_angle, wind_speed = 0, 2
    G.defineWind(wind_angle,wind_speed)
    
#-----------------Uncomment the solving strategy-------------------------------
    
    # Random strategy
#    G.addEdgesAll()
#    G.solveRandom(200000,show_evolution=True)
#    G.plotPath(gradual=True)
    
    # Loop strategy
#    G.addEdgesAll()
#    G.solveLoop(30,show_evolution=True)
#    G.plotPath(gradual=True)

    # Nearest Neighbour Strategy with a Delaunay triangulation
    G.addEdgesDelaunay()
    G.solveNearestNeighbour()
    G.plotPath(gradual=True)
    
    
    # Genetic algorithm strategy
#    G.addEdgesAll()
#    G.solveGenetic(temperature = 1000000, pop_size = 25, show_evolution=True)
#    G.plotPath(gradual=True)
    
#-----------------------------------------------------------------------------

    # Converting the path for the map
    mapPath = [np.array([[200*list(G.vertices[k])[0]],[200*list(G.vertices[k])[1]]]) for k in G.path]
    start = mapPath[0]
    end = mapPath[-1]
    
    # Constructing the autonomous sailboat
    x0= array([[-5,-5,-3,0.2,0]]).T  #x=(x,y,θ,v,w)
    a_tw = wind_speed    # true wind force
    ψ_tw = wind_angle    # true wind angle
    r = 10      # maximale acceptable distance from target line
    ζ = pi/4    # closed hauled angle for the no-go zone
    δrmax = 1   # maximal rudder angle
    β = pi/4    # angle of the sail in crosswind 
    B = Boat(x0,a_tw, ψ_tw, r, ζ, δrmax, β)
    
    # Matplotlib parameters    
    t = 0
    dt = 0.1
    ax=init_figure(-10,210,-10,210)
    measuring = True
    last_point_reached = False


    while measuring:
        B.nextStep(ax,mapPath,dt,showTrajectory=True,plot_frequence=5)        
        if not(last_point_reached):
            last_point_reached = ((end[0,0]==mapPath[0][0,0]) and (end[1,0]==mapPath[0][1,0]))
        else :
            if ((start[0,0]==mapPath[0][0,0]) and (start[1,0]==mapPath[0][1,0])):
                measuring = False
                break;
    plt.show()
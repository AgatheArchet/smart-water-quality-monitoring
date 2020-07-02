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
from scipy.integrate import solve_ivp


def readData(filename):
    lat, lon = [],[] 
    with open(filename, 'r') as f:
        if f.readline()=='#beginning\n':
            start = (f.readline()).replace(","," ").replace(";"," ").replace(
                    "\n","").split()
            beginning = [float(start[0]),float(start[1])]
            f.readline()
        for line in f:
            points = line.replace(","," ").replace(";"," ").replace("\n",""
                                 ).split()
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
    
def normalizeObject(list_x,list_y, min_X, max_X, min_Y,max_Y):
    list_x_norm = [(x-min_X)/(max_X-min_X) for x in list_x]
    list_y_norm = [(y-min_Y)/(max_Y-min_Y) for y in list_y]
    return(list_x_norm,list_y_norm)
    
def euclideanDistance(xa,xb,ya,yb):
    return(m.sqrt((yb-ya)**2+(xb-xa)**2))
    
def extractDuplicate(a):
    b = list(set(a))
    for elem in b:
        a.remove(elem)
    return(a)
      
if __name__=='__main__':
    
#----------------Uncomment and customize the area's shape----------------------

    #Grid    
    # Genarating points, optional if all points are already known
#    GPSpoints = np.array([[50.352473,-4.161624],[50.352473,-4.134152],
#                          [50.343769,-4.134152],[50.343769,-4.161624],
#                          [50.352473,-4.161624]])
#    A = Area(100,GPSpoints[0,:],GPSpoints,"grid")
#    A.placeMeasurementPoints()
#    A.generateFile()
#    
#    # Reading points from the file
#    begining,lat,lon = readData("points.txt")
#    x_list_utm,y_list_utm = LatLonToUTMXY(lat,lon)
#    x_list,y_list = normalize(x_list_utm,y_list_utm)
    
    
    #Simple Grid
    nb_of_points = 100
    GPSpoints = np.array([[0,0],[0,100],[100,100],[100,0],[0,0]])
    A = Area(nb_of_points,GPSpoints[0,:],GPSpoints,"grid")
    A.placeMeasurementPoints()
    x_listRaw = A.points_lat.reshape(nb_of_points).tolist()
    y_listRaw = A.points_lon.reshape(nb_of_points).tolist()
    x_list,y_list = normalize(x_listRaw,y_listRaw)
    
    # Simple Circle
#    center, beginning = [1,1],[4,4]
#    C = Area(65,beginning,beginning,"circle",center, angle_division=16)
#    C.placeMeasurementPoints()
#    x_listRaw,y_listRaw = C.points_lat, C.points_lon
#    x_list,y_list = normalize(x_listRaw,y_listRaw)
    
#----------------Specify the characteristics of the wind-----------------------    
    
    # Creating a graph
    G = Graph()
    G.addVertices(x_list,y_list)
    wind_angle, wind_speed = -pi/2, 2
    G.defineWind(wind_angle,wind_speed)
    
#----------------Uncomment the edges creation method-----------------------
    
    # Enable all possible edges between each existing vertex
#    G.addEdgesAll()
    
    # Enable only edges that link vertices with their nearest neighbour
    # Preferable when adding an obstacle to drastically lower the complexity
    G.addEdgesDelaunay()

#-----Uncomment and add obstacle's coordinates, choose a safety distance--------
    
    obx,oby = [2.5,7.5,7.5,5,2.5],[0.1,0.1,15,17,15]
    obx,oby = normalizeObject(obx,oby,min(x_listRaw),max(x_listRaw),
                                  min(y_listRaw),max(y_listRaw))
    G.addPolygoneObstacleAtCoords(obx,oby,safety_distance = 0.012)
    plt.plot(obx+[obx[0]],oby+[oby[0]],"blue")
    G.plot()
    
#-----------------Uncomment the solving strategy-------------------------------
    
    # Random strategy
#    G.solveRandom(200000,show_evolution=True)
#    G.plotPath(gradual=True)
    
    # Loop strategy
#    G.solveLoop(30,show_evolution=True)
#    G.plotPath(gradual=True)

    # Nearest Neighbour Strategy with a Delaunay triangulation
    G.solveNearestNeighbour()
    G.plotPath(gradual=True, obstacle_x = obx, obstacle_y = oby)
    
    
    # Genetic algorithm strategy
#    G.solveGenetic(temperature = 1000000, pop_size = 25, show_evolution=True)
#    G.plotPath(gradual=True)
    
    
#---------------Complete with the boat's characteristics-----------------------

    # Converting the path for the map
#    mapPath = [np.array([[300*list(G.vertices[k])[0]],
#                          [300*list(G.vertices[k])[1]]]) for k in G.path]
#    start = mapPath[0]
#    end = mapPath[-1]
    
    # Constructing the autonomous sailboat
#    x0= array([[300,125,-pi,0.2,0]]).T  #x=(x,y,θ,v,w)
#    a_tw = wind_speed    # true wind force
#    ψ_tw = wind_angle    # true wind angle
#    r = 10      # maximale acceptable distance from target line
#    ζ = pi/4    # closed hauled angle for the no-go zone
#    δrmax = 1   # maximal rudder angle
#    β = pi/4    # angle of the sail in crosswind 
#    B = Boat(x0,a_tw, ψ_tw, r, ζ, δrmax, β)
    
    # Matplotlib parameters    
#    dt = 0.1
#    ax=init_figure(mapPath)
#    plot_frequency = 15
#    end_index = len(mapPath)
    
#-----------------Uncomment the numerical integration--------------------------
    
    # Euler
#    while True:
#        B.nextStep(ax,mapPath,dt,showTrajectory=True,plot_frequency=min(20,plot_frequency))  
#
#        no = np.where(array(mapPath).reshape(len(mapPath),2) == end.T)[0]
#        new_index = extractDuplicate(no.tolist())[0]
#        if (new_index-end_index)<=0:
#            print(new_index-end_index)
#            end_index = new_index
#        else:
#            B.nextStep(ax,mapPath,dt,showTrajectory=True,plot_frequency=min(20,plot_frequency)) 
#            break
                
    # Runge-Kutta 45
#    while True:
#        y = solve_ivp(f_ode_45, (0,plot_frequency),(x0.T).tolist()[0],method='RK45',
#                      args=(B,ax,mapPath,True))
#        x0 = y.y[:,-1].reshape((5,1))
#        no = np.where(array(mapPath).reshape(len(mapPath),2) == end.T)[0]
#        new_index = extractDuplicate(no.tolist())[0]
#        print(new_index)
#        if (new_index-end_index)<=0:
#            print(new_index-end_index)
#            end_index = new_index
#        else:
#            break
#            y = solve_ivp(f_ode_45, (0,100),(x0.T).tolist()[0],method='RK45',
#                          args=(B,ax,mapPath,True))
#    
    plt.show()
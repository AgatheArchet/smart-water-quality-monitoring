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
from measurement_points_generator import Area
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
    
def normalization(list_x,list_y):
    max_X = max(list_x)
    max_Y = max(list_y)
    list_x_norm = [x/max_X for x in list_x]
    list_y_norm = [y/max_Y for y in list_y]
    return(list_x_norm,list_y_norm)
        

#50.352473, -4.161624
#50.343769, -4.134152
GPSpoints = np.array([[50.352473,-4.161624],[50.352473,-4.134152],[50.343769,-4.134152],[50.343769,-4.161624],[50.352473,-4.161624]])
A = Area(100,GPSpoints[0,:],GPSpoints,"square")
A.placeMeasurementPoints()
A.generateFile()


begining,lat,lon = readData("points.txt")
x,y = LatLonToUTMXY(lat,lon)
x,y = normalization(x,y)
points = np.array([x,y]).T

#TO DO Delaunay ? with https://github.com/jmespadero/pyDelaunay2D
tri = Delaunay(points)
plt.triplot(points[:,0], points[:,1], tri.simplices.copy())

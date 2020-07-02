#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 12 14:02:06 2020

@author: agathe
"""
import math as m
import matplotlib.pyplot as plt
import numpy as np
from random import seed,random
import time
from scipy.spatial import ConvexHull
from copy import deepcopy

class Area:
    """
    A class that caracterizes the measurement area where the autonomous sailboat 
    must go.
    
    Attributes
    ----------
    nb_of_measuremnts : int
        number of measurements to make in the area.
    delimitations_points : 2x1 numpy array
        array of main delimitation points of the area, with points described as 
        [latitude,longitude].
    shape : string
        general shape of the path wanted.
    center : tuple of integer
        center of area if it has a circle shape
    angle division : integer (power of two)
        precision for circle shaped area
    
    Methods
    -------
    
    
    """
    def __init__(self, nb_of_measurement_points, beginning, delimitation_points, 
                 shape, center=None, angle_division=8):
         self.nb_points = nb_of_measurement_points 
         self.contour = delimitation_points
         self.beginning = beginning
         self.shape = shape
         self.points_lat = None
         self.points_lon = None
         self.center = center
         self.angle_div = angle_division
         
    def placeMeasurementPoints(self):
        """
        generates points accordingly to the area's constraints and properties.
        """
        if self.shape=="grid":
            min_lat, max_lat = np.min(self.contour[:,0]), np.max(self.contour[:,0])
            min_lon, max_lon = np.min(self.contour[:,1]), np.max(self.contour[:,1])
            n = int(m.sqrt(self.nb_points))
            lat_list = np.linspace(min_lat,max_lat,n)
            lon_list = np.linspace(min_lon,max_lon,n)
            self.points_lat, self.points_lon = np.meshgrid(lat_list,lon_list) 
            
        if self.shape=="circle":
             self.points_lat,self.points_lon = [],[]
             radius = m.sqrt((self.beginning[0]-self.center[0])**2+(self.beginning[1]-self.center[1])**2)
             angleStart = m.atan2(self.beginning[1]-self.center[1],self.beginning[0]-self.center[0])
             self.points_lat.append(self.center[0])
             self.points_lon.append(self.center[1])
             points_package = (self.nb_points-1)//self.angle_div
             for point in range(0,points_package):
                 for angle in range(self.angle_div):
                     x = (1-point/points_package)*radius*m.cos(angle/self.angle_div*2*m.pi-angleStart)+self.center[0]
                     y = (1-point/points_package)*radius*m.sin(angle/self.angle_div*2*m.pi-angleStart)+self.center[1]
                     self.points_lat.append(x)
                     self.points_lon.append(y)
                     
        if self.shape =="coastal":
            self.points_lat,self.points_lon = [],[]
            x_poly, y_poly = self.contour[:,0],self.contour[:,1]
            maxi,mini = max(max(x_poly),max(y_poly)),min(min(x_poly),min(y_poly))
            seed(time.time())
            for i in range(self.nb_points):
                print(i)
                while True:
                    x,y = random()*(maxi-mini)+mini, random()*(maxi-mini)+mini
                    if isInsidePolygon(len(x_poly),x_poly,y_poly,x,y):
                        self.points_lat.append(x)
                        self.points_lon.append(y)
                        break
                     
            
            
    def generateMap(self):
        """
        plots a graph of the points for a clear representation of the area. 
        """
        if self.shape=="grid" or self.shape=="coastal":
            plt.plot(self.contour[:,0],self.contour[:,1],'g--')
        if self.shape=="round":
            radius = m.sqrt((self.beginning[0]-self.center[0])**2+(self.beginning[1]-self.center[1])**2)
            plt.Circle((self.center[0], self.center[1]), radius, color='b', fill=False)
        plt.plot(self.points_lat,self.points_lon,'oc')
        plt.plot(self.beginning[0],self.beginning[1],'ro')
        plt.show()
    
    def generateFile(self):
        """
        writes the area's points into a file for futher use.
        """
        f = open("points.txt","w")
        f.write("#beginning\n")
        f.write('{},{},\n'.format(self.beginning[0],self.beginning[1]))
        f.write("#points\n")
        if self.shape=="grid":
            for i in range(len(self.points_lat)):
                for j in range(len(self.points_lon)):
                    f.write('{},{};\n'.format(self.points_lat[0,i],self.points_lon[j,0]))
        if self.shape=="circle":
            for i in range(len(self.points_lat)):
                    f.write('{:.2f},{:.2f};\n'.format(self.points_lat[i],self.points_lon[i]))
        f.close()
        
def isInsidePolygon(n,Vx,Vy,x,y):
    """
    verifies if a given point (x,y) is inside any polygon (based upon Jordan 
    curve theorem).
    """
    c = False
    for i in range(0,n):
        j = (i-1)%n
        if (((Vy[i]>y)!=(Vy[j]>y)) and 
            (x < (Vx[j]-Vx[i])*(y-Vy[i])/(Vy[j]-Vy[i])+Vx[i])):
            c = not(c)
    return c     
        
if __name__=='__main__':
    
#    GPSpoints = np.array([[1,1],[1,6],[6,6],[6,1],[1,1]])
#    A = Area(100,GPSpoints[0,:],GPSpoints,"grid")
#    A.placeMeasurementPoints()
#    A.generateFile()
#    A.generateMap()

#    center, beginning = [1,1],[4,4]
#    C = Area(65,beginning,beginning,"circle",center, angle_division=16)
#    C.placeMeasurementPoints()
#    C.generateFile()
#    C.generateMap()
    
    GPSpoints = np.array([[1,1],[10,1],[10,10],[9,9],[8,9],[7,10.8],[6,10.8],[6.5,10],[6,8],[5,11],[4,9],[3,8.5],[2,11],[1,10],[1,1]])
    Coast = Area(100,GPSpoints[0,:],GPSpoints,"coastal")
    Coast.placeMeasurementPoints()
    Coast.generateMap()
    
    hull = ConvexHull(GPSpoints)
    for simplex in hull.simplices:
        plt.plot(GPSpoints[simplex, 0], GPSpoints[simplex, 1], '-')
        
    convexMap = deepcopy(GPSpoints).tolist()
    for simplex in hull.simplices:
        print(GPSpoints[simplex[0]])
#        if GPSpoints[simplex[0]] in convexMap:
#            convexMap.remove(GPSpoints[simplex[0]])
#        if GPSpoints[simplex[1]] in convexMap:
#            convexMap.remove(GPSpoints[simplex[1]])

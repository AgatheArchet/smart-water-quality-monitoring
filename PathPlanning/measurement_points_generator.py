#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 12 14:02:06 2020

@author: agathe
"""
import math as m
import matplotlib.pyplot as plt
import numpy as np

class Area:
    """
    A class to caracterize the measurement area where the autonomous sailboat 
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
    
    Methods
    -------
    
    
    """
    def __init__(self, nb_of_measurement_points, beginning, delimitation_points, shape):
         self.nb_points = nb_of_measurement_points 
         self.contour = delimitation_points
         self.beginning = beginning
         self.shape = shape
         self.points_lat = None
         self.points_lon = None
         
    def placeMeasurementPoints(self):
        if self.shape=="square":
            min_lat, max_lat = np.min(self.contour[:,0]), np.max(self.contour[:,0])
            min_lon, max_lon = np.min(self.contour[:,1]), np.max(self.contour[:,1])
            n = int(m.sqrt(self.nb_points))
            lat_list = np.linspace(min_lat,max_lat,n)
            lon_list = np.linspace(min_lon,max_lon,n)
            self.points_lat, self.points_lon = np.meshgrid(lat_list,lon_list) 
            
    def generateMap(self):
        ax = plt.gca()
        ax.set_facecolor("#2C7FCA")
        plt.plot(self.contour[:,0],self.contour[:,1],'g--')
        plt.plot(self.points_lat,self.points_lon,'oc')
        plt.plot(self.beginning[0],self.beginning[1],'ro')
        plt.show()
    
    def generateFile(self):
        f = open("points.txt","w")
        f.write("#beginning\n")
        f.write('{},{},\n'.format(self.beginning[0],self.beginning[1]))
        f.write("#points\n")
        for i in range(len(self.points_lat)):
            for j in range(len(self.points_lon)):
                f.write('{},{};\n'.format(self.points_lat[0,i],self.points_lon[j,0]))
        f.close()
    
        
if __name__=='__main__':
    GPSpoints = np.array([[1,1],[1,6],[6,6],[6,1],[1,1]])
    A = Area(100,GPSpoints[0,:],GPSpoints,"square")
    A.placeMeasurementPoints()
    A.generateFile()
    A.generateMap()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 15 10:32:40 2020

@author: agathe
"""

class Graph:
    """
    A class to modelize an graph of various verticies.
    
    Attributes
    ----------
    coords_list : organized list of tuple
        the coordinates of all vertices, with the i-th element the coordoonates
        (x,y) for the vertex number i.
    self.dist : dictionnary with two keys
        the key (u,v) with float value d indicates the distance d to go 
        from vextex u to v.If there is no existing distance between two 
        vertices, they are unreachables.
    path : list of integers
        the list with the optimal known path
    wind_angle : integer betwenn -180 and 180
        current direction of the wind, with respect to unit circle convention
        north wind : -90° / east wind : 0° / south wind : 90°
    
    """
    def __init__(self):
        self.coords_list = []
        self.dist = {}
        self.path = None
        self.wind_angle = None
        
    def addVertex(self,x,y):
        self.coords_list.append((x,y))
        
    def addEdge(self, from_node, to_node, weight):
        self.weights[(from_node, to_node)] = weight
    
    def getAssociatedNumber(x,y):
        return(self.coords_list.index((x,y)))
        
        
            
        

    
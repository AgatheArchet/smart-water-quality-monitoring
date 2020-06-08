#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wen May 27 11:33:34 2020

@author: agathe

A kalman filter for measuremenst coming from arduino via BLE.
To modify errors model of sensors, modify tools in initKalmanFilter() method:
    * Γx : estimated error on initial state
    * Γα = np.diag([0.5,0.5]) # estimated error on model
    * Γβ = np.diag([0.8,0.8]) # estimated error on each mesaurement
It is possible to ask for displaying the results by enabling "lists" boolean 
when initialiazing and updating the kalman filer.
It is possible to simulate values and measurements by enabling "simulatedValues"
when updating the kalman filter. Their behavior can then be changed in update()
method.

"""

import numpy as np
import matplotlib.pyplot as plt

def kalman_predict(xup,Gup,u,Γα,A):
    """
    Predicts the values of state vector xup (second step of the Kalman filter).
    """
    Γ1 = A @ Gup @ A.T + Γα
    x1 = A @ xup + u    
    return(x1,Γ1)    

def kalman_correc(x0,Γ0,y,Γβ,C):
    """
    Corrects the values of state vector x0 (first step of the Kalman filter).
    """
    S = C @ Γ0 @ C.T + Γβ        
    K = Γ0 @ C.T @ np.linalg.inv(S)           
    ytilde = y - C @ x0        
    Gup = (np.eye(len(x0))-K @ C) @ Γ0 
    xup = x0 + K@ytilde
    return(xup,Gup) 
    
def kalman(x0,Γ0,u,y,Γα,Γβ,A,C):
    """
    Corrects and predicts the right values of state vector x using 
    the Kalman filter.
    """
    xup,Gup = kalman_correc(x0,Γ0,y,Γβ,C)
    x1,Γ1=kalman_predict(xup,Gup,u,Γα,A)
    return(x1,Γ1) 
    
def update(xnew, simulation = False):
    """
    Updates values of state vector x and observation vector y with
    data collected from simulation or BLE.
    """
    # to collect to update, or simulaton with gaussian noises
    if simulation==True:
            x = xnew + np.array([[1],[1]])*(np.random.random()-0.5)
            y = np.array([[x[0,0,]],[x[1,0]]]) + np.random.normal(0,0.5,(2,1))
    else:
        x = xnew
        y = np.array([[x[0,0,]],[x[1,0]]])
    return(x,y)
    
def initKalmanFilter(firstPhValue,firstEcValue,lists=False):
    """
    Initializes all variables to use the Kalman filter.
    """
    # model :  x = A*x + u 
    x = np.array([[firstPhValue],[firstEcValue]]) #simulated or collected from arduino
    Γx = np.diag([0.2,0.2]) # estimated error on initial state
    u = 0 # no command
    A = np.eye(2)
    
    # measurements : y = C*x 
    y = np.array([[x[0,0]],[x[1,0]]])
    C = np.eye(2)
    
    #  estimation : x_ = A*x + u + α / y_ = C*x_ + β
    xhat = np.array([[x[0,0]],[x[1,0]]])
    Γα = np.diag([0.5,0.5]) # estimated error on model
    Γβ = np.diag([0.8,0.8]) # estimated error on each mesaurement
    
    if lists==True:
        # graphical tools
        list_x = [x]
        list_y = [y]
        list_xhat = [xhat]
        return(x,Γx,u,A,y,C,xhat,Γα,Γβ,list_x,list_y,list_xhat)
    else:
        return(x,Γx,u,A,y,C,xhat,Γα,Γβ)
    
    
def updateKalmanFilter(xnew,y,xhat,u,Γx,Γα,Γβ,A,C,lists=False,list_x=[],list_y=[],list_xhat=[], simulatedValues=False):
    """
    Collects new values measured and applies them to the Kalman filter.
    """
    x,y = update(xnew, simulatedValues)
    xhat, Γx = kalman(xhat,Γx,u,y,Γα,Γβ,A,C)
    if lists==True:
        list_x.append(x)
        list_y.append(y)
        list_xhat.append(xhat)
        return(x,y,xhat,Γx,list_x,list_y,list_xhat)
    else:
        return(x,y,xhat,Γx)
    
def plotKalmanGraph(timeMax,list_x,list_y,list_xhat):
    """
    Shows comparative graphs if results were saved (enable list option) 
    """
    abscissa = np.arange(0, timeMax+1, 2)
    plt.subplot(2, 1, 1)
    plt.plot(abscissa, np.array(list_x)[:,0,0], 'g.', label="true")
    plt.plot(abscissa, np.array(list_y)[:,0,0], 'c', label="measured")
    plt.plot(abscissa, np.array(list_xhat)[:,0,0],'r',label="estimed")
    plt.title('Kalman filter applied to sensors as a function of time')
    plt.ylabel('pH')
    plt.legend()
    
    plt.subplot(2, 1, 2)
    plt.plot(abscissa, np.array(list_x)[:,1,0], 'g.',label="true")
    plt.plot(abscissa, np.array(list_y)[:,1,0],'c',label="measured")
    plt.plot(abscissa,  np.array(list_xhat)[:,1,0],'r',label="estimed")
    plt.ylabel('Condictivity (mS/cm)')
    
    plt.legend()
    plt.show()


if __name__ == '__main__':
    
    #with graphical results
    dataSensors,Γsensors,u,A,measurements,C,dataSensorsEstimed,Γα,Γβ,list_dataTrue,list_dataMeasured,list_dataEstimed = initKalmanFilter(7.13,7.86, True)
    time = 0
    timeMax = 150
    while(time < timeMax):
        time += 2
        dataSensors, measurements, dataSensorsEstimed, Γsensors, list_dataTrue, list_dataMeasured, list_dataEstimed = updateKalmanFilter(dataSensors,measurements,dataSensorsEstimed,0,Γsensors,Γα,Γβ,A,C,True, list_dataTrue,list_dataMeasured,list_dataEstimed, True)
      
    plotKalmanGraph(timeMax, list_dataTrue,list_dataMeasured, list_dataEstimed)
    
    #without graphical results
#    dataSensors,Γsensors,u,A,measurements,C,dataSensorsEstimed,Γα,Γβ = initKalmanFilter(7.13,2.36)
#    time = 0
#    timeMax = 150
#    while(time < timeMax):
#        time += 2
#        dataSensors, measurements, dataSensorsEstimed, Γsensors = updateKalmanFilter(dataSensors,measurements,dataSensorsEstimed,0,Γsensors,Γα,Γβ,A,C)
#    print("done")
